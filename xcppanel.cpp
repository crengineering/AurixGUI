#include "xcppanel.h"
#include "plotwidget.h"
#include "lampicon.h"

#include <QCheckBox>
#include <QDateTime>
#include <QFileDialog>
#include <QLineEdit>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QHeaderView>
#include <QTabWidget>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QtEndian>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QSet>
#include <QSettings>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <cstring>
#include <functional>

namespace {
// Fixed addresses, see Measurements.h / Diagnostics.h in the firmware.
constexpr quint32 XCP_DATA_ADDR = 0x70030000;
constexpr int     POLL_MS       = 100;  // 10 Hz

// Persistent parameter block Xcp_Nvm (firmware >= v1.6.0), strictly
// separated from the RAM-only cal block. Only this block is stored in the
// DFLASH. The master writes SAVE/DFLT into the command word, the firmware
// executes it in its 100 ms task and clears the word back to 0 (handshake).
constexpr quint32 XCP_NVM_ADDR    = 0x70030200;
constexpr quint32 NVM_CMD_ADDR    = XCP_NVM_ADDR + 0x04;
constexpr quint32 NVM_PARAM_ADDR  = XCP_NVM_ADDR + 0x08;
constexpr quint32 NVM_CMD_SAVE    = 0x45564153;   // "SAVE"
constexpr quint32 NVM_CMD_DFLT    = 0x544C4644;   // "DFLT"
constexpr int     NVM_POLL_MS     = 150;
constexpr int     NVM_MAX_RETRIES = 10;

// Persistent parameters (uint32 each, offsets after the command word).
// Only parameters that are deliberately persistent appear here; everything
// else belongs in the RAM cal block (A2L-driven "Calibration" tab).
struct NvmParam { const char *key; const char *name; };
constexpr NvmParam NVM_PARAMS[] = {
    {"userValue", "userValue - free uint32 (validation)"},
    {"seaLevelPa", "seaLevelPa - sea-level ref [Pa] for baro altitude (QNH)"},
};
constexpr int NVM_VALUES = int(sizeof(NVM_PARAMS) / sizeof(NVM_PARAMS[0]));

// diagStatus bits, must match DIAGNOSTICS.md
struct DiagBit { int bit; const char *text; };
constexpr DiagBit DIAG_BITS[] = {
    {0,  "DTS temperature too low (PMS sensor)"},
    {1,  "DTS temperature too high (PMS sensor)"},
    {2,  "DTSC temperature too low (SCU sensor)"},
    {3,  "DTSC temperature too high (SCU sensor)"},
    {4,  "VDD undervoltage (1.25 V core)"},
    {5,  "VDD overvoltage (1.25 V core)"},
    {6,  "VDDP3 undervoltage (3.3 V I/O)"},
    {7,  "VDDP3 overvoltage (3.3 V I/O)"},
    {8,  "VEXT undervoltage (5 V board)"},
    {9,  "VEXT overvoltage (5 V board)"},
    {10, "Temperature sensors implausible (DTS/DTSC delta)"},
    {11, "UART link disconnected (no heartbeat from PC)"},
    {12, "NVM fault (DFLASH corrupt or save failed)"},
    {31, "Calibration block invalid - defaults reloaded"},
};
constexpr int DIAG_ROWS = int(sizeof(DIAG_BITS) / sizeof(DIAG_BITS[0]));
constexpr int DIAG_TAB_INDEX = 1;   // "Diagnostics" position in the sub-tabs

// Master list of channels the user can log to MF4. Each carries an accessor
// from a received measurement frame; isFloat=false selects a uint32 column
// (tick/diag). Pressure is logged in hPa to match the live/plot display.
struct LogChannel {
    QString name;
    QString unit;
    bool    isFloat;
    std::function<double(const XcpClient::Measurements &)> get;
};

const QVector<LogChannel> &logChannels()
{
    static const QVector<LogChannel> chans = {
        {"DieTemp_DTS",     "\xC2\xB0""C", true,  [](const XcpClient::Measurements &m){ return double(m.dieTempC); }},
        {"DieTemp_DTSC",    "\xC2\xB0""C", true,  [](const XcpClient::Measurements &m){ return double(m.dtscTempC); }},
        {"VDD",             "V",           true,  [](const XcpClient::Measurements &m){ return double(m.vddCore); }},
        {"VDDP3",           "V",           true,  [](const XcpClient::Measurements &m){ return double(m.vddp3); }},
        {"VEXT",            "V",           true,  [](const XcpClient::Measurements &m){ return double(m.vext); }},
        {"BaroPressure",    "hPa",         true,  [](const XcpClient::Measurements &m){ return double(m.baroPressPa) / 100.0; }},
        {"BaroTemperature", "\xC2\xB0""C", true,  [](const XcpClient::Measurements &m){ return double(m.baroTempC); }},
        {"BaroAltitude",    "m",           true,  [](const XcpClient::Measurements &m){ return double(m.baroAltM); }},
        {"TickMs",          "ms",          false, [](const XcpClient::Measurements &m){ return double(m.tickMs); }},
        {"DiagStatus",      QString(),     false, [](const XcpClient::Measurements &m){ return double(m.diagStatus); }},
    };
    return chans;
}

// The calibration table is now built from the A2L CHARACTERISTIC list
// (see A2lModel). These helpers decode/encode a cell value per record type.

QString formatCharValue(const A2lChar &c, const uchar *p)
{
    switch (c.type) {
    case A2lType::Float32: {
        float f;
        std::memcpy(&f, p, sizeof(f));
        return QString::number(double(f), 'g', 6);
    }
    case A2lType::Uint32:
        return QString::number(qFromLittleEndian<quint32>(p));
    case A2lType::Uint8:
        return QString::number(uint(p[0]));
    }
    return QStringLiteral("-");
}

QByteArray encodeCharValue(const A2lChar &c, const QString &text, bool *ok)
{
    QByteArray out;
    *ok = false;
    const QString s = text.trimmed();
    switch (c.type) {
    case A2lType::Float32: {
        const float f = QString(s).replace(',', '.').toFloat(ok);
        if (!*ok) break;
        out.resize(4);
        std::memcpy(out.data(), &f, sizeof(f));
        break;
    }
    case A2lType::Uint32: {
        const quint32 v = s.toUInt(ok, 0);   // base 0: "12" or "0x0C"
        if (!*ok) break;
        out.resize(4);
        qToLittleEndian<quint32>(v, out.data());
        break;
    }
    case A2lType::Uint8: {
        const uint v = s.toUInt(ok, 0);
        if (*ok && v > 255u) *ok = false;
        if (!*ok) break;
        out.resize(1);
        out[0] = char(uchar(v));
        break;
    }
    }
    return out;
}
}

XcpPanel::XcpPanel(QWidget *parent)
    : QWidget(parent)
{
    m_client = new XcpClient(this);

    // ---- Connection row (shared above the sub-tabs) ----
    m_hostEdit = new QLineEdit("192.168.0.10");
    m_portBox  = new QSpinBox;
    m_portBox->setRange(1, 65535);
    m_portBox->setValue(5555);
    m_connectBtn = new QPushButton("Connect");
    m_identLbl   = new QLabel("-");

    auto *topRow = new QHBoxLayout;
    topRow->addWidget(new QLabel("IP:"));
    topRow->addWidget(m_hostEdit, 1);
    topRow->addWidget(new QLabel("Port:"));
    topRow->addWidget(m_portBox);
    topRow->addWidget(m_connectBtn);
    topRow->addWidget(new QLabel("Board:"));
    topRow->addWidget(m_identLbl);

    // Load the firmware's A2L up front so the Calibration tab is built from
    // it. Widgets do not exist yet, so loadA2l only fills m_chars here; the
    // table + status label are populated when buildCalTab() runs below.
    loadA2l(defaultA2lPath(), /*remember=*/false);

    m_subTabs = new QTabWidget;
    m_subTabs->addTab(buildLiveTab(), "Live Data");
    m_subTabs->addTab(buildDiagTab(), "Diagnostics");
    m_subTabs->addTab(buildCalTab(),  "Calibration");
    m_subTabs->addTab(buildNvmTab(),  "DFLASH");
    m_subTabs->addTab(buildPlotTab(), "Plot && Log");
    updateDiagLamp();

    auto *layout = new QVBoxLayout(this);
    layout->addLayout(topRow);
    layout->addWidget(m_subTabs, 1);

    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(POLL_MS);

    connect(m_connectBtn, &QPushButton::clicked, this, &XcpPanel::toggleConnection);
    connect(m_pollTimer,  &QTimer::timeout,      this, &XcpPanel::pollTick);
    connect(m_client, &XcpClient::connected,            this, &XcpPanel::onConnected);
    connect(m_client, &XcpClient::disconnected,         this, &XcpPanel::onDisconnected);
    connect(m_client, &XcpClient::measurementsReceived, this, &XcpPanel::onMeasurements);
    connect(m_client, &XcpClient::memoryRead,           this, &XcpPanel::onMemoryRead);
    connect(m_client, &XcpClient::memoryWritten,        this, &XcpPanel::onMemoryWritten);
    connect(m_client, &XcpClient::daqStarted,           this, &XcpPanel::onDaqStarted);
    connect(m_client, &XcpClient::daqFailed,            this, &XcpPanel::onDaqFailed);
    connect(m_client, &XcpClient::errorOccurred,        this, &XcpPanel::onError);

    // connect to the board automatically on startup (with the default
    // host/port); a failed attempt just logs a timeout and can be retried
    QTimer::singleShot(0, this, &XcpPanel::toggleConnection);
}

QWidget *XcpPanel::buildLiveTab()
{
    m_versionLbl = new QLabel("-");
    m_uptimeLbl  = new QLabel("-");
    m_tempLbl    = new QLabel("-");
    m_dtscLbl    = new QLabel("-");
    m_vddLbl     = new QLabel("-");
    m_vddp3Lbl   = new QLabel("-");
    m_vextLbl    = new QLabel("-");
    m_baroPressLbl = new QLabel("-");
    m_baroTempLbl  = new QLabel("-");
    m_baroAltLbl   = new QLabel("-");

    QFont bigFont = m_tempLbl->font();
    bigFont.setPointSize(bigFont.pointSize() * 2);
    bigFont.setBold(true);
    m_tempLbl->setFont(bigFont);

    auto *form = new QFormLayout;
    form->addRow("Software version:",       m_versionLbl);
    form->addRow("Uptime:",                 m_uptimeLbl);
    form->addRow("Die temperature (DTS):",  m_tempLbl);
    form->addRow("Die temperature (DTSC):", m_dtscLbl);
    form->addRow("VDD 1.25 V:",             m_vddLbl);
    form->addRow("VDDP3 3.3 V:",            m_vddp3Lbl);
    form->addRow("VEXT 5 V:",               m_vextLbl);
    form->addRow("Baro pressure (BMP388):", m_baroPressLbl);
    form->addRow("Baro temperature:",       m_baroTempLbl);
    form->addRow("Baro altitude:",          m_baroAltLbl);

    m_log = new QPlainTextEdit;
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(500);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addLayout(form);
    layout->addWidget(m_log, 1);
    return tab;
}

QWidget *XcpPanel::buildDiagTab()
{
    m_diagWordLbl = new QLabel("diagStatus: -");
    QFont mono("Consolas");
    mono.setStyleHint(QFont::Monospace);
    m_diagWordLbl->setFont(mono);

    m_diagSummary = new QLabel("Not connected");
    QFont bold = m_diagSummary->font();
    bold.setBold(true);
    bold.setPointSize(bold.pointSize() + 2);
    m_diagSummary->setFont(bold);

    // Overall error lamp: one glance instead of scanning the bit table.
    m_diagLamp = new QLabel;
    m_diagLamp->setPixmap(lampPixmap(LampColor::Gray, 18));

    m_diagTable = new QTableWidget(DIAG_ROWS, 3);
    m_diagTable->setHorizontalHeaderLabels({"Bit", "Meaning", "Status"});
    m_diagTable->verticalHeader()->setVisible(false);
    m_diagTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_diagTable->setSelectionMode(QAbstractItemView::NoSelection);
    for (int i = 0; i < DIAG_ROWS; ++i) {
        m_diagTable->setItem(i, 0, new QTableWidgetItem(QString::number(DIAG_BITS[i].bit)));
        m_diagTable->setItem(i, 1, new QTableWidgetItem(QString::fromUtf8(DIAG_BITS[i].text)));
        m_diagTable->setItem(i, 2, new QTableWidgetItem("-"));
    }
    m_diagTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);

    auto *summaryRow = new QHBoxLayout;
    summaryRow->addWidget(m_diagLamp);
    summaryRow->addWidget(m_diagSummary, 1);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addLayout(summaryRow);
    layout->addWidget(m_diagWordLbl);
    layout->addWidget(m_diagTable, 1);
    layout->addWidget(new QLabel("See DIAGNOSTICS.md in the firmware repository for details."));
    return tab;
}

QWidget *XcpPanel::buildCalTab()
{
    m_calTable = new QTableWidget(0, 4);
    m_calTable->setHorizontalHeaderLabels({"Parameter", "Unit", "Value", ""});
    m_calTable->verticalHeader()->setVisible(false);
    m_calTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    m_calReadBtn   = new QPushButton("Read all");
    m_calExportBtn = new QPushButton("Export...");
    m_calImportBtn = new QPushButton("Import...");
    m_a2lLoadBtn   = new QPushButton("Load A2L...");
    m_calExportBtn->setToolTip("Save the parameter set (table values) to a JSON file");
    m_calImportBtn->setToolTip("Load a parameter set from a JSON file into the table - "
                               "values are then written individually per parameter");
    m_a2lLoadBtn->setToolTip("Load a different A2L file; the table is rebuilt from its "
                             "CHARACTERISTIC list");
    m_calReadBtn->setEnabled(false);
    connect(m_calReadBtn,   &QPushButton::clicked, this, &XcpPanel::readCalibration);
    connect(m_calExportBtn, &QPushButton::clicked, this, &XcpPanel::exportCalibration);
    connect(m_calImportBtn, &QPushButton::clicked, this, &XcpPanel::importCalibration);
    connect(m_a2lLoadBtn,   &QPushButton::clicked, this, &XcpPanel::loadA2lFile);

    m_a2lStatus = new QLabel("-");

    auto *btnRow = new QHBoxLayout;
    btnRow->addWidget(m_calReadBtn);
    btnRow->addWidget(m_a2lLoadBtn);
    btnRow->addStretch(1);
    btnRow->addWidget(m_calImportBtn);
    btnRow->addWidget(m_calExportBtn);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addWidget(m_a2lStatus);
    layout->addWidget(m_calTable, 1);
    layout->addLayout(btnRow);
    layout->addWidget(new QLabel("Rows come from the A2L (CHARACTERISTIC list). "
                                 "RAM working page - defaults apply again after a reset. "
                                 "Persistent parameters: \"DFLASH\" tab."));

    rebuildCalTable();      // fills the table from m_chars loaded in the ctor
    return tab;
}

// (Re)build the calibration rows from the parsed A2L characteristics. Row i
// maps to m_chars[i], so read/write need no extra lookup table.
void XcpPanel::rebuildCalTable()
{
    if (!m_calTable)
        return;

    m_calTable->setRowCount(0);     // drops old items and cell widgets
    m_calRowBtns.clear();
    m_calTable->setRowCount(m_chars.size());

    const bool connected = m_client->isConnected();
    for (int i = 0; i < m_chars.size(); ++i) {
        const A2lChar &c = m_chars[i];

        auto *name = new QTableWidgetItem(c.name);
        name->setFlags(name->flags() & ~Qt::ItemIsEditable);
        if (!c.desc.isEmpty())
            name->setToolTip(c.desc);
        auto *unit = new QTableWidgetItem(c.unit);
        unit->setFlags(unit->flags() & ~Qt::ItemIsEditable);
        m_calTable->setItem(i, 0, name);
        m_calTable->setItem(i, 1, unit);
        m_calTable->setItem(i, 2, new QTableWidgetItem("-"));   // editable

        auto *writeBtn = new QPushButton("Write");
        writeBtn->setEnabled(connected);
        connect(writeBtn, &QPushButton::clicked, this, [this, i]() { writeCalRow(i); });
        m_calTable->setCellWidget(i, 3, writeBtn);
        m_calRowBtns.append(writeBtn);
    }

    if (m_a2lStatus) {
        if (m_chars.isEmpty())
            m_a2lStatus->setText("No A2L loaded - use \"Load A2L...\"");
        else
            m_a2lStatus->setText(QString("Loaded %1 characteristics from %2")
                                     .arg(m_chars.size())
                                     .arg(QDir::toNativeSeparators(m_a2lPath)));
    }
}

// Candidate A2L: last one used (QSettings), else the sibling firmware repo.
QString XcpPanel::defaultA2lPath() const
{
    QSettings settings;
    const QString saved = settings.value("a2lPath").toString();
    if (!saved.isEmpty() && QFile::exists(saved))
        return saved;

    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        appDir + "/../../AurixTricore/docs/AurixTricore.a2l",   // exe in build/
        appDir + "/../AurixTricore/docs/AurixTricore.a2l",
        appDir + "/AurixTricore.a2l",
    };
    for (const QString &c : candidates) {
        if (QFile::exists(c))
            return QFileInfo(c).absoluteFilePath();
    }
    return saved;                   // may be empty -> "No A2L loaded"
}

void XcpPanel::loadA2l(const QString &path, bool remember)
{
    QString err;
    const QVector<A2lChar> parsed = A2lModel::parseFile(path, &err);
    if (parsed.isEmpty()) {
        if (m_log)
            m_log->appendPlainText("[A2L load failed: " + err + " (" + path + ")]");
        if (m_a2lStatus && m_chars.isEmpty())
            m_a2lStatus->setText("No A2L loaded - " + err);
        return;                     // keep any previously loaded characteristics
    }

    // The persistent NVM block keeps its own DFLASH tab, so drop it here.
    m_chars.clear();
    for (const A2lChar &c : parsed) {
        if (c.addr >= XCP_NVM_ADDR && c.addr < XCP_NVM_ADDR + 0x100u)
            continue;
        m_chars.append(c);
    }
    m_a2lPath = path;

    if (remember) {
        QSettings settings;
        settings.setValue("a2lPath", path);
    }
    if (m_log)
        m_log->appendPlainText(QString("[A2L loaded: %1 characteristics from %2]")
                                   .arg(m_chars.size()).arg(path));
    rebuildCalTable();
}

void XcpPanel::loadA2lFile()
{
    const QString path = QFileDialog::getOpenFileName(
        this, "Load A2L description", m_a2lPath, "A2L (*.a2l);;All files (*)");
    if (path.isEmpty())
        return;
    loadA2l(path, /*remember=*/true);
}

QWidget *XcpPanel::buildNvmTab()
{
    m_nvmTable = new QTableWidget(NVM_VALUES, 3);
    m_nvmTable->setHorizontalHeaderLabels({"Parameter", "Value", ""});
    m_nvmTable->verticalHeader()->setVisible(false);
    for (int i = 0; i < NVM_VALUES; ++i) {
        auto *name = new QTableWidgetItem(QString::fromUtf8(NVM_PARAMS[i].name));
        name->setFlags(name->flags() & ~Qt::ItemIsEditable);
        m_nvmTable->setItem(i, 0, name);
        m_nvmTable->setItem(i, 1, new QTableWidgetItem("-"));   // editable

        auto *writeBtn = new QPushButton("Write");
        writeBtn->setEnabled(false);
        connect(writeBtn, &QPushButton::clicked, this, [this, i]() { writeNvmRow(i); });
        m_nvmTable->setCellWidget(i, 2, writeBtn);
        m_nvmRowBtns.append(writeBtn);
    }
    m_nvmTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    m_nvmReadBtn = new QPushButton("Read all");
    m_nvmSaveBtn = new QPushButton("Save to DFLASH");
    m_nvmDfltBtn = new QPushButton("Load defaults");
    m_nvmSaveBtn->setToolTip("Persist the NVM block in the DFLASH, surviving resets (SAVE)");
    m_nvmDfltBtn->setToolTip("Load the NVM defaults into RAM - does not save (DFLT)");
    m_nvmReadBtn->setEnabled(false);
    m_nvmSaveBtn->setEnabled(false);
    m_nvmDfltBtn->setEnabled(false);
    connect(m_nvmReadBtn, &QPushButton::clicked, this, &XcpPanel::readNvmParams);
    connect(m_nvmSaveBtn, &QPushButton::clicked, this, &XcpPanel::saveNvmToFlash);
    connect(m_nvmDfltBtn, &QPushButton::clicked, this, &XcpPanel::loadNvmDefaults);

    m_nvmStatus = new QLabel("-");

    auto *btnRow = new QHBoxLayout;
    btnRow->addWidget(m_nvmReadBtn);
    btnRow->addStretch(1);
    btnRow->addWidget(m_nvmDfltBtn);
    btnRow->addWidget(m_nvmSaveBtn);

    auto *statusRow = new QHBoxLayout;
    statusRow->addWidget(new QLabel("Last command:"));
    statusRow->addWidget(m_nvmStatus, 1);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addWidget(new QLabel("Persistent parameters (Xcp_Nvm @ 0x70030200) - only this block "
                                 "is stored in the DFLASH. Only parameters that were deliberately "
                                 "created as persistent appear here."));
    layout->addWidget(m_nvmTable, 1);
    layout->addLayout(btnRow);
    layout->addLayout(statusRow);
    layout->addWidget(new QLabel("Storage health: see diagnostic bit 12 (NVM fault)."));
    return tab;
}

QWidget *XcpPanel::buildPlotTab()
{
    m_plot = new PlotWidget;
    // Pressure (~956 hPa) shares the common autoscaled y-axis, so it is default
    // off like the volts — toggle it on its own for a readable trend; baro temp
    // overlays the die temps (same scale).
    const struct { const char *name; QColor color; bool on; } defs[8] = {
        {"DTS [°C]",     QColor(200, 60, 60),  true},
        {"DTSC [°C]",    QColor(230, 140, 40), true},
        {"VDD [V]",      QColor(60, 120, 200), false},
        {"VDDP3 [V]",    QColor(60, 180, 90),  false},
        {"VEXT [V]",     QColor(140, 80, 200), false},
        {"Baro P [hPa]", QColor(0, 160, 160),  false},
        {"Baro T [°C]",  QColor(170, 40, 120), false},
        {"Baro alt [m]", QColor(90, 90, 90),   false},
    };

    auto *chkRow = new QHBoxLayout;
    for (int i = 0; i < 8; ++i) {
        m_series[i]  = m_plot->addSeries(QString::fromUtf8(defs[i].name), defs[i].color);
        m_plotChk[i] = new QCheckBox(QString::fromUtf8(defs[i].name));
        m_plotChk[i]->setChecked(defs[i].on);
        m_plot->setSeriesVisible(m_series[i], defs[i].on);
        const int idx = i;
        connect(m_plotChk[i], &QCheckBox::toggled, this, [this, idx](bool on) {
            m_plot->setSeriesVisible(m_series[idx], on);
        });
        chkRow->addWidget(m_plotChk[i]);
    }
    chkRow->addStretch(1);

    m_logStartBtn = new QPushButton("Start Log");
    m_logStopBtn  = new QPushButton("Stop Log");
    m_logSaveBtn  = new QPushButton("Save...");
    m_logStatus   = new QLabel("No log");
    m_logStartBtn->setEnabled(false);
    m_logStopBtn->setEnabled(false);
    m_logSaveBtn->setEnabled(false);
    connect(m_logStartBtn, &QPushButton::clicked, this, &XcpPanel::startLogging);
    connect(m_logStopBtn,  &QPushButton::clicked, this, &XcpPanel::stopLogging);
    connect(m_logSaveBtn,  &QPushButton::clicked, this, &XcpPanel::saveLogging);

    auto *logRow = new QHBoxLayout;
    logRow->addWidget(m_logStartBtn);
    logRow->addWidget(m_logStopBtn);
    logRow->addWidget(m_logSaveBtn);
    logRow->addWidget(m_logStatus, 1);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addLayout(chkRow);
    layout->addWidget(m_plot, 1);
    layout->addLayout(logRow);
    return tab;
}

void XcpPanel::toggleConnection()
{
    if (m_client->isConnected()) {
        m_client->disconnectFromSlave();
    } else {
        m_log->appendPlainText(QString("[Connecting to %1:%2 ...]")
                                   .arg(m_hostEdit->text())
                                   .arg(m_portBox->value()));
        m_client->connectToSlave(m_hostEdit->text(), quint16(m_portBox->value()));
    }
}

void XcpPanel::onConnected(const QString &ident)
{
    m_identLbl->setText(ident.isEmpty() ? "(unknown)" : ident);
    m_log->appendPlainText("[Connected: " + m_identLbl->text() + "]");
    setConnectedState(true);
    m_timeBase.restart();
    m_plot->clearData();
    m_lastDaqMs = 0;
    m_pollTimer->start();
    readCalibration();                          // populate the calibration tab
    m_client->pollMeasurements(XCP_DATA_ADDR);  // one poll: version + magic
    m_client->startDaq(XCP_DATA_ADDR + 8, 44);  // then event-driven streaming
}

void XcpPanel::onDaqStarted()
{
    m_log->appendPlainText("[DAQ started - measurements arrive event-driven (100 ms)]");
}

void XcpPanel::onDaqFailed()
{
    m_log->appendPlainText("[DAQ not available - falling back to polling]");
}

void XcpPanel::onDisconnected()
{
    m_pollTimer->stop();
    setConnectedState(false);
    m_log->appendPlainText("[Disconnected]");
}

void XcpPanel::onMeasurements(const XcpClient::Measurements &m)
{
    if (!m.valid) {
        m_log->appendPlainText("[Warning: magic word mismatch - firmware incompatible with this GUI?]");
        return;
    }

    m_versionLbl->setText(QString("%1.%2.%3")
                              .arg(m.verMajor).arg(m.verMinor).arg(m.verStep));

    const quint32 s = m.tickMs / 1000;
    m_uptimeLbl->setText(QString("%1:%2:%3")
                             .arg(s / 3600, 2, 10, QChar('0'))
                             .arg((s / 60) % 60, 2, 10, QChar('0'))
                             .arg(s % 60, 2, 10, QChar('0')));

    m_tempLbl->setText(QString::number(double(m.dieTempC), 'f', 1) + " °C");
    m_dtscLbl->setText(QString::number(double(m.dtscTempC), 'f', 1) + " °C");
    m_vddLbl->setText(QString::number(double(m.vddCore), 'f', 3) + " V");
    m_vddp3Lbl->setText(QString::number(double(m.vddp3), 'f', 3) + " V");
    m_vextLbl->setText(QString::number(double(m.vext), 'f', 3) + " V");

    if (m.baroPresent) {
        // show hPa (1 hPa = 100 Pa) — the familiar barometric unit
        m_baroPressLbl->setText(QString::number(double(m.baroPressPa) / 100.0, 'f', 2) + " hPa");
        m_baroTempLbl->setText(QString::number(double(m.baroTempC), 'f', 1) + " °C");
        m_baroAltLbl->setText(QString::number(double(m.baroAltM), 'f', 1) + " m (vs 1013.25 hPa)");
    } else {
        m_baroPressLbl->setText("n/a (not detected)");
        m_baroTempLbl->setText("n/a");
        m_baroAltLbl->setText("n/a");
    }

    updateDiagTable(m.diagStatus);

    // plot + logging feed
    const double t = double(m_timeBase.elapsed()) / 1000.0;
    m_lastDaqMs = m_timeBase.elapsed();
    m_plot->appendPoint(m_series[0], t, double(m.dieTempC));
    m_plot->appendPoint(m_series[1], t, double(m.dtscTempC));
    m_plot->appendPoint(m_series[2], t, double(m.vddCore));
    m_plot->appendPoint(m_series[3], t, double(m.vddp3));
    m_plot->appendPoint(m_series[4], t, double(m.vext));
    if (m.baroPresent) {
        m_plot->appendPoint(m_series[5], t, double(m.baroPressPa) / 100.0);  // hPa
        m_plot->appendPoint(m_series[6], t, double(m.baroTempC));
        m_plot->appendPoint(m_series[7], t, double(m.baroAltM));
    }

    if (m_logging) {
        const auto &chans = logChannels();
        QVector<double> values;
        values.reserve(m_logSel.size());
        for (int idx : m_logSel)
            values.append(chans[idx].get(m));
        m_mf4.append(double(m_timeBase.elapsed() - m_logStartMs) / 1000.0, values);
        updateLogStatus();
    }
}

// Modal picker shown before each logging session. Returns false if the user
// cancelled or left everything unchecked; otherwise fills *out with the chosen
// channel indices (into logChannels()).
bool XcpPanel::chooseLogChannels(QVector<int> *out)
{
    const auto &chans = logChannels();

    QDialog dlg(this);
    dlg.setWindowTitle("Select channels to log");
    auto *v = new QVBoxLayout(&dlg);
    v->addWidget(new QLabel("Choose which measurement values to record:"));

    QVector<QCheckBox *> boxes;
    for (int i = 0; i < chans.size(); ++i) {
        auto *cb = new QCheckBox(chans[i].name
                                 + (chans[i].unit.isEmpty() ? QString()
                                                            : " [" + chans[i].unit + "]"));
        // default: all on for the first session, else remember the last choice
        cb->setChecked(m_logSel.isEmpty() ? true : m_logSel.contains(i));
        boxes.append(cb);
        v->addWidget(cb);
    }

    auto *selAllBtn = new QPushButton("Select all");
    connect(selAllBtn, &QPushButton::clicked, &dlg, [&boxes]() {
        for (QCheckBox *b : boxes) b->setChecked(true);
    });

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto *btnRow = new QHBoxLayout;
    btnRow->addWidget(selAllBtn);
    btnRow->addStretch(1);
    btnRow->addWidget(buttons);
    v->addLayout(btnRow);

    if (dlg.exec() != QDialog::Accepted)
        return false;

    out->clear();
    for (int i = 0; i < boxes.size(); ++i)
        if (boxes[i]->isChecked())
            out->append(i);
    return !out->isEmpty();
}

void XcpPanel::startLogging()
{
    QVector<int> sel;
    if (!chooseLogChannels(&sel)) {
        m_log->appendPlainText("[Logging not started - cancelled or no channels selected]");
        return;
    }
    m_logSel = sel;

    const auto &chans = logChannels();
    QVector<Mf4Writer::Channel> cfg;
    cfg.reserve(sel.size());
    for (int idx : sel)
        cfg.append({chans[idx].name, chans[idx].unit, chans[idx].isFloat});
    m_mf4.begin(cfg);

    m_logStartMs = m_timeBase.elapsed();
    m_logging    = true;
    m_logStartBtn->setEnabled(false);
    m_logStopBtn->setEnabled(true);
    m_logSaveBtn->setEnabled(false);
    m_log->appendPlainText(QString("[Logging started - %1 channels]").arg(sel.size()));
    updateLogStatus();
}

void XcpPanel::stopLogging()
{
    m_logging = false;
    m_logStartBtn->setEnabled(m_client->isConnected());
    m_logStopBtn->setEnabled(false);
    m_logSaveBtn->setEnabled(m_mf4.count() > 0);
    m_log->appendPlainText(QString("[Logging stopped: %1 samples]").arg(m_mf4.count()));
    updateLogStatus();
}

void XcpPanel::saveLogging()
{
    const QString suggested = "aurix_log_"
        + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".mf4";
    const QString path = QFileDialog::getSaveFileName(
        this, "Save MF4 log", suggested, "ASAM MDF 4 (*.mf4)");
    if (path.isEmpty())
        return;

    QString err;
    if (m_mf4.save(path, &err))
        m_log->appendPlainText("[MF4 saved: " + path + "]");
    else
        m_log->appendPlainText("[MF4 save failed: " + err + "]");
}

void XcpPanel::updateLogStatus()
{
    if (m_logging) {
        const double secs = double(m_timeBase.elapsed() - m_logStartMs) / 1000.0;
        m_logStatus->setText(QString("Recording: %1 samples (%2 s)")
                                 .arg(m_mf4.count())
                                 .arg(secs, 0, 'f', 1));
    } else if (m_mf4.count() > 0) {
        m_logStatus->setText(QString("Log ready to save: %1 samples")
                                 .arg(m_mf4.count()));
    } else {
        m_logStatus->setText("No log");
    }
}

void XcpPanel::updateDiagTable(quint32 status)
{
    if (m_haveStatus && status == m_lastStatus)
        return;                                  // nothing changed

    m_diagWordLbl->setText("diagStatus: 0x"
                           + QString("%1").arg(status, 8, 16, QChar('0')).toUpper());

    if (status == 0) {
        m_diagSummary->setText("Board OK");
        m_diagSummary->setStyleSheet("color: green;");
    } else {
        m_diagSummary->setText("ERROR detected");
        m_diagSummary->setStyleSheet("color: red;");
    }

    for (int i = 0; i < DIAG_ROWS; ++i) {
        const bool active = (status >> DIAG_BITS[i].bit) & 1u;
        QTableWidgetItem *item = m_diagTable->item(i, 2);
        item->setText(active ? "ERROR" : "OK");
        item->setForeground(active ? QBrush(Qt::red) : QBrush(Qt::darkGreen));

        // log newly appearing problems with their plain-text meaning
        const bool wasActive = m_haveStatus && ((m_lastStatus >> DIAG_BITS[i].bit) & 1u);
        if (active && !wasActive)
            m_log->appendPlainText("[Diagnostic: " + QString::fromUtf8(DIAG_BITS[i].text) + "]");
        else if (!active && wasActive)
            m_log->appendPlainText("[Diagnostic cleared: " + QString::fromUtf8(DIAG_BITS[i].text) + "]");
    }

    m_lastStatus = status;
    m_haveStatus = true;
    updateDiagLamp();
}

// Error lamp in the Diagnostics tab plus the matching sub-tab icon:
// green = connected and no error bit set, red = at least one error,
// gray = not connected (state unknown).
void XcpPanel::updateDiagLamp()
{
    QColor color = LampColor::Gray;
    if (m_haveStatus)
        color = (m_lastStatus == 0) ? LampColor::Green : LampColor::Red;

    m_diagLamp->setPixmap(lampPixmap(color, 18));
    m_subTabs->setTabIcon(DIAG_TAB_INDEX, lampIcon(color));
}

void XcpPanel::readCalibration()
{
    if (m_chars.isEmpty())
        return;

    // Group characteristics by their 0x100 block and issue one read per block
    // (each block spans <= ~60 bytes, well within a single SHORT_UPLOAD).
    QSet<quint32> bases;
    for (const A2lChar &c : m_chars)
        bases.insert(c.addr & 0xFFFFFF00u);

    for (quint32 base : bases) {
        quint32 lo = 0xFFFFFFFFu;
        quint32 hi = 0;
        for (const A2lChar &c : m_chars) {
            if ((c.addr & 0xFFFFFF00u) != base)
                continue;
            lo = qMin(lo, c.addr);
            hi = qMax(hi, c.addr + quint32(a2lTypeSize(c.type)));
        }
        if (hi > lo) {
            quint32 span = hi - lo;
            if (span > 63u)
                span = 63u;         // SHORT_UPLOAD payload cap
            m_client->readMemory(lo, quint8(span));
        }
    }
}

// ---- NVM command handshake -------------------------------------------
// Write SAVE/DFLT into the command word, then poll it until the firmware
// clears it (executed) or the retries run out.

void XcpPanel::saveNvmToFlash()
{
    sendNvmCommand(NVM_CMD_SAVE, "SAVE");
}

void XcpPanel::loadNvmDefaults()
{
    sendNvmCommand(NVM_CMD_DFLT, "DFLT");
}

void XcpPanel::sendNvmCommand(quint32 cmd, const QString &name)
{
    if (!m_nvmCmdName.isEmpty())
        return;                     // one command at a time

    m_nvmCmdName = name;
    m_nvmRetries = NVM_MAX_RETRIES;
    setNvmBusy(true);

    QByteArray word(4, '\0');
    qToLittleEndian<quint32>(cmd, word.data());
    m_client->writeMemory(NVM_CMD_ADDR, word);
    m_nvmStatus->setText(name + " running...");
    m_log->appendPlainText("[NVM: " + name + " sent]");
}

void XcpPanel::pollNvmCommand()
{
    m_client->readMemory(NVM_CMD_ADDR, 4);
}

void XcpPanel::setNvmBusy(bool busy)
{
    const bool enable = m_client->isConnected() && !busy;
    m_nvmSaveBtn->setEnabled(enable);
    m_nvmDfltBtn->setEnabled(enable);
}

void XcpPanel::readNvmParams()
{
    m_client->readMemory(NVM_PARAM_ADDR, quint8(NVM_VALUES * 4));
}

void XcpPanel::writeNvmRow(int row)
{
    bool          ok    = false;
    const QString text  = m_nvmTable->item(row, 1)->text().trimmed();
    const quint32 value = text.toUInt(&ok, 0);   // base 0: "123" or "0x7B"
    if (!ok) {
        m_log->appendPlainText(QString("[Invalid value for %1 - aborted]")
                               .arg(QString::fromUtf8(NVM_PARAMS[row].key)));
        return;
    }

    QByteArray word(4, '\0');
    qToLittleEndian<quint32>(value, word.data());
    m_client->writeMemory(NVM_PARAM_ADDR + quint32(row) * 4, word);
}

void XcpPanel::onMemoryRead(quint32 address, const QByteArray &data)
{
    if (address == NVM_CMD_ADDR && data.size() >= 4 && !m_nvmCmdName.isEmpty()) {
        const quint32 word = qFromLittleEndian<quint32>(data.constData());
        if (word == 0) {
            m_log->appendPlainText("[NVM: " + m_nvmCmdName
                                   + " executed - status: see diagnostic bit 12]");
            m_nvmStatus->setText(m_nvmCmdName + " executed");
            m_nvmCmdName.clear();
            setNvmBusy(false);
            readNvmParams();    // DFLT changes values; after SAVE a no-op refresh
        } else if (--m_nvmRetries > 0) {
            QTimer::singleShot(NVM_POLL_MS, this, &XcpPanel::pollNvmCommand);
        } else {
            m_log->appendPlainText("[NVM: " + m_nvmCmdName
                                   + " not acknowledged - firmware < v1.6.0?]");
            m_nvmStatus->setText(m_nvmCmdName + " not acknowledged");
            m_nvmCmdName.clear();
            setNvmBusy(false);
        }
        return;
    }

    if (address == NVM_PARAM_ADDR && data.size() >= NVM_VALUES * 4) {
        for (int i = 0; i < NVM_VALUES; ++i) {
            const quint32 v = qFromLittleEndian<quint32>(data.constData() + i * 4);
            m_nvmTable->item(i, 1)->setText(QString::number(v));
        }
        m_log->appendPlainText("[NVM parameters read]");
        return;
    }

    // A2L characteristic block read-back: fill any row whose value lies
    // inside the returned window (cal floats and GPIO bytes alike).
    populateCharsFromRead(address, data);
}

// Populate value cells for every characteristic contained in [base, base+len).
// Returns true if at least one cell was updated.
bool XcpPanel::populateCharsFromRead(quint32 base, const QByteArray &data)
{
    bool any = false;
    for (int i = 0; i < m_chars.size(); ++i) {
        const A2lChar &c = m_chars[i];
        if (c.addr < base)
            continue;
        const int off = int(c.addr - base);
        if (off + a2lTypeSize(c.type) > data.size())
            continue;
        const uchar *p = reinterpret_cast<const uchar *>(data.constData()) + off;
        if (m_calTable->item(i, 2))
            m_calTable->item(i, 2)->setText(formatCharValue(c, p));
        any = true;
    }
    if (any)
        m_log->appendPlainText("[Calibration/GPIO values read]");
    return any;
}

void XcpPanel::writeCalRow(int row)
{
    if (row < 0 || row >= m_chars.size())
        return;

    const A2lChar &c = m_chars[row];
    bool ok = false;
    const QByteArray bytes = encodeCharValue(c, m_calTable->item(row, 2)->text(), &ok);
    if (!ok) {
        m_log->appendPlainText(QString("[Invalid value for %1 - aborted]").arg(c.name));
        return;
    }

    m_client->writeMemory(c.addr, bytes);
}

void XcpPanel::exportCalibration()
{
    const QString path = QFileDialog::getSaveFileName(
        this, "Export parameter set", "calibration.json", "JSON (*.json)");
    if (path.isEmpty())
        return;

    QJsonObject obj;
    for (int i = 0; i < m_chars.size(); ++i) {
        bool  ok = false;
        const double v = m_calTable->item(i, 2)->text().replace(',', '.').toDouble(&ok);
        if (!ok) {
            m_log->appendPlainText(QString("[Export aborted: invalid value for %1]")
                                   .arg(m_chars[i].name));
            return;
        }
        obj.insert(m_chars[i].name, v);
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        m_log->appendPlainText("[Export failed: " + file.errorString() + "]");
        return;
    }
    file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    m_log->appendPlainText("[Parameter set exported: " + path + "]");
}

void XcpPanel::importCalibration()
{
    const QString path = QFileDialog::getOpenFileName(
        this, "Import parameter set", QString(), "JSON (*.json)");
    if (path.isEmpty())
        return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        m_log->appendPlainText("[Import failed: " + file.errorString() + "]");
        return;
    }

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        m_log->appendPlainText("[Import failed: not a valid JSON object]");
        return;
    }

    const QJsonObject obj = doc.object();
    int applied = 0;
    for (int i = 0; i < m_chars.size(); ++i) {
        const A2lChar &c = m_chars[i];
        if (obj.contains(c.name) && obj.value(c.name).isDouble()) {
            const double d = obj.value(c.name).toDouble();
            const QString s = (c.type == A2lType::Float32)
                                  ? QString::number(d, 'g', 6)
                                  : QString::number(qint64(d));
            m_calTable->item(i, 2)->setText(s);
            ++applied;
        }
    }
    m_log->appendPlainText(QString("[Parameter set imported: %1 of %2 values - "
                                   "write them individually to apply]")
                           .arg(applied).arg(m_chars.size()));
}

void XcpPanel::onMemoryWritten(quint32 address)
{
    if (address == NVM_CMD_ADDR) {
        // command word landed on the board; give the 100 ms task one cycle
        QTimer::singleShot(NVM_POLL_MS, this, &XcpPanel::pollNvmCommand);
        return;
    }

    if (address >= NVM_PARAM_ADDR && address < NVM_PARAM_ADDR + quint32(NVM_VALUES) * 4) {
        const int row = int((address - NVM_PARAM_ADDR) / 4);
        m_log->appendPlainText(QString("[%1 written - remember \"Save to DFLASH\"]")
                               .arg(QString::fromUtf8(NVM_PARAMS[row].key)));
        readNvmParams();        // read back for confirmation
        return;
    }

    for (const A2lChar &c : m_chars) {
        if (address == c.addr) {
            m_log->appendPlainText(QString("[%1 written]").arg(c.name));
            readCalibration();      // read back for confirmation
            return;
        }
    }
}

void XcpPanel::onError(const QString &message)
{
    m_log->appendPlainText("[Error: " + message + "]");
}

void XcpPanel::pollTick()
{
    if (m_client->daqActive()) {
        // DAQ streams by itself; the timer only watches for silence
        if (m_lastDaqMs > 0 && m_timeBase.elapsed() - m_lastDaqMs > 1500) {
            m_log->appendPlainText("[DAQ stream lost]");
            m_client->disconnectFromSlave();
        }
        return;
    }
    m_client->pollMeasurements(XCP_DATA_ADDR);
}

void XcpPanel::setConnectedState(bool connected)
{
    m_connectBtn->setText(connected ? "Disconnect" : "Connect");
    m_hostEdit->setEnabled(!connected);
    m_portBox->setEnabled(!connected);
    m_calReadBtn->setEnabled(connected);
    for (QPushButton *btn : m_calRowBtns)
        btn->setEnabled(connected);
    m_nvmReadBtn->setEnabled(connected);
    m_nvmSaveBtn->setEnabled(connected);
    m_nvmDfltBtn->setEnabled(connected);
    for (QPushButton *btn : m_nvmRowBtns)
        btn->setEnabled(connected);
    if (!connected) {
        m_nvmCmdName.clear();   // abandon a pending NVM handshake
        m_nvmStatus->setText("-");
    }

    if (!connected && m_logging)
        stopLogging();
    m_logStartBtn->setEnabled(connected && !m_logging);
    m_logStopBtn->setEnabled(connected && m_logging);
    m_logSaveBtn->setEnabled(!m_logging && m_mf4.count() > 0);

    if (!connected) {
        m_versionLbl->setText("-");
        m_uptimeLbl->setText("-");
        m_tempLbl->setText("-");
        m_dtscLbl->setText("-");
        m_vddLbl->setText("-");
        m_vddp3Lbl->setText("-");
        m_vextLbl->setText("-");
        m_baroPressLbl->setText("-");
        m_baroTempLbl->setText("-");
        m_baroAltLbl->setText("-");
        m_identLbl->setText("-");
        m_diagSummary->setText("Not connected");
        m_diagSummary->setStyleSheet("");
        m_diagWordLbl->setText("diagStatus: -");
        m_haveStatus = false;
        for (int i = 0; i < DIAG_ROWS; ++i) {
            m_diagTable->item(i, 2)->setText("-");
            m_diagTable->item(i, 2)->setForeground(QBrush());
        }
        updateDiagLamp();
    }

    emit connectionChanged(connected);
}
