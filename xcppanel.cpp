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
#include <cstring>

namespace {
// Fixed addresses, see Measurements.h / Diagnostics.h in the firmware.
constexpr quint32 XCP_DATA_ADDR = 0x70030000;
constexpr quint32 XCP_CAL_ADDR  = 0x70030100;
constexpr int     CAL_VALUES    = 15;   // floats after the magic word
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
// else belongs in the cal block / CAL_PARAMS.
struct NvmParam { const char *key; const char *name; };
constexpr NvmParam NVM_PARAMS[] = {
    {"userValue", "userValue - freies uint32 (Validierung)"},
};
constexpr int NVM_VALUES = int(sizeof(NVM_PARAMS) / sizeof(NVM_PARAMS[0]));

// diagStatus bits, must match DIAGNOSTICS.md
struct DiagBit { int bit; const char *text; };
constexpr DiagBit DIAG_BITS[] = {
    {0,  "DTS-Temperatur zu niedrig (PMS-Sensor)"},
    {1,  "DTS-Temperatur zu hoch (PMS-Sensor)"},
    {2,  "DTSC-Temperatur zu niedrig (SCU-Sensor)"},
    {3,  "DTSC-Temperatur zu hoch (SCU-Sensor)"},
    {4,  "VDD-Unterspannung (1,25 V Kern)"},
    {5,  "VDD-Überspannung (1,25 V Kern)"},
    {6,  "VDDP3-Unterspannung (3,3 V I/O)"},
    {7,  "VDDP3-Überspannung (3,3 V I/O)"},
    {8,  "VEXT-Unterspannung (5 V Board)"},
    {9,  "VEXT-Überspannung (5 V Board)"},
    {10, "Temperatursensoren unplausibel (Delta DTS/DTSC)"},
    {11, "UART-Verbindung getrennt (kein Heartbeat vom PC)"},
    {12, "NVM-Fehler (DFLASH korrupt oder Speichern fehlgeschlagen)"},
    {31, "Kalibrierblock ungültig - Defaults geladen"},
};
constexpr int DIAG_ROWS = int(sizeof(DIAG_BITS) / sizeof(DIAG_BITS[0]));
constexpr int DIAG_TAB_INDEX = 1;   // "Diagnose" position in the sub-tabs

// calibration block layout (offset after magic = index * 4); the key is
// the stable identifier used for JSON export/import
struct CalParam { const char *key; const char *name; const char *unit; };
constexpr CalParam CAL_PARAMS[CAL_VALUES] = {
    {"dtsMin",       "dtsMin   - DTS Untergrenze",        "°C"},
    {"dtsMax",       "dtsMax   - DTS Obergrenze",         "°C"},
    {"dtscMin",      "dtscMin  - DTSC Untergrenze",       "°C"},
    {"dtscMax",      "dtscMax  - DTSC Obergrenze",        "°C"},
    {"vddMin",       "vddMin   - VDD Untergrenze",        "V"},
    {"vddMax",       "vddMax   - VDD Obergrenze",         "V"},
    {"vddp3Min",     "vddp3Min - VDDP3 Untergrenze",      "V"},
    {"vddp3Max",     "vddp3Max - VDDP3 Obergrenze",       "V"},
    {"vextMin",      "vextMin  - VEXT Untergrenze",       "V"},
    {"vextMax",      "vextMax  - VEXT Obergrenze",        "V"},
    {"tempDeltaMax", "tempDeltaMax - max. Sensor-Delta",  "K"},
    {"debounceSec",  "debounceSec  - Entprellzeit",       "s"},
    {"fsVdd",        "fsVdd    - ADC-Endwert VDD",        "V"},
    {"fsVddp3",      "fsVddp3  - ADC-Endwert VDDP3",      "V"},
    {"fsVext",       "fsVext   - ADC-Endwert VEXT",       "V"},
};
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

    m_subTabs = new QTabWidget;
    m_subTabs->addTab(buildLiveTab(), "Messwerte");
    m_subTabs->addTab(buildDiagTab(), "Diagnose");
    m_subTabs->addTab(buildCalTab(),  "Kalibrierung");
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

    QFont bigFont = m_tempLbl->font();
    bigFont.setPointSize(bigFont.pointSize() * 2);
    bigFont.setBold(true);
    m_tempLbl->setFont(bigFont);

    auto *form = new QFormLayout;
    form->addRow("Software-Version:",       m_versionLbl);
    form->addRow("Uptime:",                 m_uptimeLbl);
    form->addRow("Die-Temperatur (DTS):",   m_tempLbl);
    form->addRow("Die-Temperatur (DTSC):",  m_dtscLbl);
    form->addRow("VDD 1.25 V:",             m_vddLbl);
    form->addRow("VDDP3 3.3 V:",            m_vddp3Lbl);
    form->addRow("VEXT 5 V:",               m_vextLbl);

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

    m_diagSummary = new QLabel("Nicht verbunden");
    QFont bold = m_diagSummary->font();
    bold.setBold(true);
    bold.setPointSize(bold.pointSize() + 2);
    m_diagSummary->setFont(bold);

    // Overall error lamp: one glance instead of scanning the bit table.
    m_diagLamp = new QLabel;
    m_diagLamp->setPixmap(lampPixmap(LampColor::Gray, 18));

    m_diagTable = new QTableWidget(DIAG_ROWS, 3);
    m_diagTable->setHorizontalHeaderLabels({"Bit", "Bedeutung", "Status"});
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
    layout->addWidget(new QLabel("Interpretation siehe DIAGNOSTICS.md im Firmware-Repository."));
    return tab;
}

QWidget *XcpPanel::buildCalTab()
{
    m_calTable = new QTableWidget(CAL_VALUES, 4);
    m_calTable->setHorizontalHeaderLabels({"Parameter", "Einheit", "Wert", ""});
    m_calTable->verticalHeader()->setVisible(false);
    for (int i = 0; i < CAL_VALUES; ++i) {
        auto *name = new QTableWidgetItem(QString::fromUtf8(CAL_PARAMS[i].name));
        name->setFlags(name->flags() & ~Qt::ItemIsEditable);
        auto *unit = new QTableWidgetItem(QString::fromUtf8(CAL_PARAMS[i].unit));
        unit->setFlags(unit->flags() & ~Qt::ItemIsEditable);
        m_calTable->setItem(i, 0, name);
        m_calTable->setItem(i, 1, unit);
        m_calTable->setItem(i, 2, new QTableWidgetItem("-"));   // editable

        // one write button per parameter: values go to the board one by
        // one and deliberately never as a bulk write
        auto *writeBtn = new QPushButton("Schreiben");
        writeBtn->setEnabled(false);
        connect(writeBtn, &QPushButton::clicked, this, [this, i]() { writeCalRow(i); });
        m_calTable->setCellWidget(i, 3, writeBtn);
        m_calRowBtns.append(writeBtn);
    }
    m_calTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    m_calReadBtn   = new QPushButton("Alle lesen");
    m_calExportBtn = new QPushButton("Exportieren...");
    m_calImportBtn = new QPushButton("Importieren...");
    m_calExportBtn->setToolTip("Parametersatz (Tabellenwerte) als JSON-Datei speichern");
    m_calImportBtn->setToolTip("Parametersatz aus JSON-Datei in die Tabelle laden - "
                               "geschrieben wird danach einzeln pro Parameter");
    m_calReadBtn->setEnabled(false);
    connect(m_calReadBtn,   &QPushButton::clicked, this, &XcpPanel::readCalibration);
    connect(m_calExportBtn, &QPushButton::clicked, this, &XcpPanel::exportCalibration);
    connect(m_calImportBtn, &QPushButton::clicked, this, &XcpPanel::importCalibration);

    auto *btnRow = new QHBoxLayout;
    btnRow->addWidget(m_calReadBtn);
    btnRow->addStretch(1);
    btnRow->addWidget(m_calImportBtn);
    btnRow->addWidget(m_calExportBtn);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addWidget(m_calTable, 1);
    layout->addLayout(btnRow);
    layout->addWidget(new QLabel("Arbeitsseite im RAM - nach einem Reset gelten wieder die Defaults. "
                                 "Persistente Parameter: Tab \"DFLASH\"."));
    return tab;
}

QWidget *XcpPanel::buildNvmTab()
{
    m_nvmTable = new QTableWidget(NVM_VALUES, 3);
    m_nvmTable->setHorizontalHeaderLabels({"Parameter", "Wert", ""});
    m_nvmTable->verticalHeader()->setVisible(false);
    for (int i = 0; i < NVM_VALUES; ++i) {
        auto *name = new QTableWidgetItem(QString::fromUtf8(NVM_PARAMS[i].name));
        name->setFlags(name->flags() & ~Qt::ItemIsEditable);
        m_nvmTable->setItem(i, 0, name);
        m_nvmTable->setItem(i, 1, new QTableWidgetItem("-"));   // editable

        auto *writeBtn = new QPushButton("Schreiben");
        writeBtn->setEnabled(false);
        connect(writeBtn, &QPushButton::clicked, this, [this, i]() { writeNvmRow(i); });
        m_nvmTable->setCellWidget(i, 2, writeBtn);
        m_nvmRowBtns.append(writeBtn);
    }
    m_nvmTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    m_nvmReadBtn = new QPushButton("Alle lesen");
    m_nvmSaveBtn = new QPushButton("Im DFLASH speichern");
    m_nvmDfltBtn = new QPushButton("Defaults laden");
    m_nvmSaveBtn->setToolTip("Persistiert den NVM-Block reset-fest im DFLASH (SAVE)");
    m_nvmDfltBtn->setToolTip("Lädt die NVM-Defaults ins RAM - speichert nicht (DFLT)");
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
    statusRow->addWidget(new QLabel("Letztes Kommando:"));
    statusRow->addWidget(m_nvmStatus, 1);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addWidget(new QLabel("Persistente Parameter (Xcp_Nvm @ 0x70030200) - nur dieser Block "
                                 "wird im DFLASH gespeichert. Hier erscheinen ausschließlich "
                                 "Parameter, die bewusst persistent angelegt wurden."));
    layout->addWidget(m_nvmTable, 1);
    layout->addLayout(btnRow);
    layout->addLayout(statusRow);
    layout->addWidget(new QLabel("Speicherstatus siehe Diagnose-Bit 12 (NVM-Fehler)."));
    return tab;
}

QWidget *XcpPanel::buildPlotTab()
{
    m_plot = new PlotWidget;
    const struct { const char *name; QColor color; bool on; } defs[5] = {
        {"DTS [°C]",   QColor(200, 60, 60),  true},
        {"DTSC [°C]",  QColor(230, 140, 40), true},
        {"VDD [V]",    QColor(60, 120, 200), false},
        {"VDDP3 [V]",  QColor(60, 180, 90),  false},
        {"VEXT [V]",   QColor(140, 80, 200), false},
    };

    auto *chkRow = new QHBoxLayout;
    for (int i = 0; i < 5; ++i) {
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
    m_logSaveBtn  = new QPushButton("Speichern...");
    m_logStatus   = new QLabel("Kein Log");
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
        m_log->appendPlainText(QString("[Verbinde mit %1:%2 ...]")
                                   .arg(m_hostEdit->text())
                                   .arg(m_portBox->value()));
        m_client->connectToSlave(m_hostEdit->text(), quint16(m_portBox->value()));
    }
}

void XcpPanel::onConnected(const QString &ident)
{
    m_identLbl->setText(ident.isEmpty() ? "(unbekannt)" : ident);
    m_log->appendPlainText("[Verbunden: " + m_identLbl->text() + "]");
    setConnectedState(true);
    m_timeBase.restart();
    m_plot->clearData();
    m_lastDaqMs = 0;
    m_pollTimer->start();
    readCalibration();                          // populate the calibration tab
    m_client->pollMeasurements(XCP_DATA_ADDR);  // one poll: version + magic
    m_client->startDaq(XCP_DATA_ADDR + 8, 32);  // then event-driven streaming
}

void XcpPanel::onDaqStarted()
{
    m_log->appendPlainText("[DAQ gestartet - Messwerte kommen event-getrieben (100 ms)]");
}

void XcpPanel::onDaqFailed()
{
    m_log->appendPlainText("[DAQ nicht verfuegbar - Fallback auf Polling]");
}

void XcpPanel::onDisconnected()
{
    m_pollTimer->stop();
    setConnectedState(false);
    m_log->appendPlainText("[Getrennt]");
}

void XcpPanel::onMeasurements(const XcpClient::Measurements &m)
{
    if (!m.valid) {
        m_log->appendPlainText("[Warnung: Magic-Word falsch - Firmware passt nicht zur GUI?]");
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

    updateDiagTable(m.diagStatus);

    // plot + logging feed
    const double t = double(m_timeBase.elapsed()) / 1000.0;
    m_lastDaqMs = m_timeBase.elapsed();
    m_plot->appendPoint(m_series[0], t, double(m.dieTempC));
    m_plot->appendPoint(m_series[1], t, double(m.dtscTempC));
    m_plot->appendPoint(m_series[2], t, double(m.vddCore));
    m_plot->appendPoint(m_series[3], t, double(m.vddp3));
    m_plot->appendPoint(m_series[4], t, double(m.vext));

    if (m_logging) {
        Mf4Writer::Sample s;
        s.t     = double(m_timeBase.elapsed() - m_logStartMs) / 1000.0;
        s.dts   = m.dieTempC;
        s.dtsc  = m.dtscTempC;
        s.vdd   = m.vddCore;
        s.vddp3 = m.vddp3;
        s.vext  = m.vext;
        s.tick  = m.tickMs;
        s.diag  = m.diagStatus;
        m_mf4.append(s);
        updateLogStatus();
    }
}

void XcpPanel::startLogging()
{
    m_mf4.clear();
    m_logStartMs = m_timeBase.elapsed();
    m_logging    = true;
    m_logStartBtn->setEnabled(false);
    m_logStopBtn->setEnabled(true);
    m_logSaveBtn->setEnabled(false);
    m_log->appendPlainText("[Logging gestartet]");
    updateLogStatus();
}

void XcpPanel::stopLogging()
{
    m_logging = false;
    m_logStartBtn->setEnabled(m_client->isConnected());
    m_logStopBtn->setEnabled(false);
    m_logSaveBtn->setEnabled(m_mf4.count() > 0);
    m_log->appendPlainText(QString("[Logging gestoppt: %1 Samples]").arg(m_mf4.count()));
    updateLogStatus();
}

void XcpPanel::saveLogging()
{
    const QString suggested = "aurix_log_"
        + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".mf4";
    const QString path = QFileDialog::getSaveFileName(
        this, "MF4-Log speichern", suggested, "ASAM MDF 4 (*.mf4)");
    if (path.isEmpty())
        return;

    QString err;
    if (m_mf4.save(path, &err))
        m_log->appendPlainText("[MF4 gespeichert: " + path + "]");
    else
        m_log->appendPlainText("[MF4 speichern fehlgeschlagen: " + err + "]");
}

void XcpPanel::updateLogStatus()
{
    if (m_logging) {
        const double secs = double(m_timeBase.elapsed() - m_logStartMs) / 1000.0;
        m_logStatus->setText(QString("Aufzeichnung laeuft: %1 Samples (%2 s)")
                                 .arg(m_mf4.count())
                                 .arg(secs, 0, 'f', 1));
    } else if (m_mf4.count() > 0) {
        m_logStatus->setText(QString("Log bereit zum Speichern: %1 Samples")
                                 .arg(m_mf4.count()));
    } else {
        m_logStatus->setText("Kein Log");
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
        m_diagSummary->setText("FEHLER erkannt");
        m_diagSummary->setStyleSheet("color: red;");
    }

    for (int i = 0; i < DIAG_ROWS; ++i) {
        const bool active = (status >> DIAG_BITS[i].bit) & 1u;
        QTableWidgetItem *item = m_diagTable->item(i, 2);
        item->setText(active ? "FEHLER" : "OK");
        item->setForeground(active ? QBrush(Qt::red) : QBrush(Qt::darkGreen));

        // log newly appearing problems with their plain-text meaning
        const bool wasActive = m_haveStatus && ((m_lastStatus >> DIAG_BITS[i].bit) & 1u);
        if (active && !wasActive)
            m_log->appendPlainText("[Diagnose: " + QString::fromUtf8(DIAG_BITS[i].text) + "]");
        else if (!active && wasActive)
            m_log->appendPlainText("[Diagnose behoben: " + QString::fromUtf8(DIAG_BITS[i].text) + "]");
    }

    m_lastStatus = status;
    m_haveStatus = true;
    updateDiagLamp();
}

// Error lamp in the Diagnose tab plus the matching sub-tab icon:
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
    // 15 floats after the magic word (60 bytes, fits one SHORT_UPLOAD)
    m_client->readMemory(XCP_CAL_ADDR + 4, CAL_VALUES * 4);
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
    m_nvmStatus->setText(name + " läuft...");
    m_log->appendPlainText("[NVM: " + name + " gesendet]");
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
        m_log->appendPlainText(QString("[Ungültiger Wert für %1 - Abbruch]")
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
                                   + " ausgeführt - Status siehe Diagnose-Bit 12]");
            m_nvmStatus->setText(m_nvmCmdName + " ausgeführt");
            m_nvmCmdName.clear();
            setNvmBusy(false);
            readNvmParams();    // DFLT changes values; after SAVE a no-op refresh
        } else if (--m_nvmRetries > 0) {
            QTimer::singleShot(NVM_POLL_MS, this, &XcpPanel::pollNvmCommand);
        } else {
            m_log->appendPlainText("[NVM: " + m_nvmCmdName
                                   + " nicht bestätigt - Firmware < v1.6.0?]");
            m_nvmStatus->setText(m_nvmCmdName + " nicht bestätigt");
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
        m_log->appendPlainText("[NVM-Parameter gelesen]");
        return;
    }

    if (address != XCP_CAL_ADDR + 4 || data.size() < CAL_VALUES * 4)
        return;

    for (int i = 0; i < CAL_VALUES; ++i) {
        float f = 0.0f;
        std::memcpy(&f, data.constData() + i * 4, sizeof(f));
        m_calTable->item(i, 2)->setText(QString::number(double(f), 'g', 6));
    }
    m_log->appendPlainText("[Kalibrierwerte gelesen]");
}

void XcpPanel::writeCalRow(int row)
{
    bool  ok = false;
    float f  = m_calTable->item(row, 2)->text().replace(',', '.').toFloat(&ok);
    if (!ok) {
        m_log->appendPlainText(QString("[Ungültiger Wert für %1 - Abbruch]")
                               .arg(QString::fromUtf8(CAL_PARAMS[row].key)));
        return;
    }

    QByteArray word(4, '\0');
    std::memcpy(word.data(), &f, sizeof(f));
    m_client->writeMemory(XCP_CAL_ADDR + 4 + quint32(row) * 4, word);
}

void XcpPanel::exportCalibration()
{
    const QString path = QFileDialog::getSaveFileName(
        this, "Parametersatz exportieren", "kalibrierung.json", "JSON (*.json)");
    if (path.isEmpty())
        return;

    QJsonObject obj;
    for (int i = 0; i < CAL_VALUES; ++i) {
        bool  ok = false;
        const double v = m_calTable->item(i, 2)->text().replace(',', '.').toDouble(&ok);
        if (!ok) {
            m_log->appendPlainText(QString("[Export abgebrochen: ungültiger Wert für %1]")
                                   .arg(QString::fromUtf8(CAL_PARAMS[i].key)));
            return;
        }
        obj.insert(QString::fromUtf8(CAL_PARAMS[i].key), v);
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        m_log->appendPlainText("[Export fehlgeschlagen: " + file.errorString() + "]");
        return;
    }
    file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
    m_log->appendPlainText("[Parametersatz exportiert: " + path + "]");
}

void XcpPanel::importCalibration()
{
    const QString path = QFileDialog::getOpenFileName(
        this, "Parametersatz importieren", QString(), "JSON (*.json)");
    if (path.isEmpty())
        return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        m_log->appendPlainText("[Import fehlgeschlagen: " + file.errorString() + "]");
        return;
    }

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        m_log->appendPlainText("[Import fehlgeschlagen: kein gültiges JSON-Objekt]");
        return;
    }

    const QJsonObject obj = doc.object();
    int applied = 0;
    for (int i = 0; i < CAL_VALUES; ++i) {
        const QString key = QString::fromUtf8(CAL_PARAMS[i].key);
        if (obj.contains(key) && obj.value(key).isDouble()) {
            m_calTable->item(i, 2)->setText(
                QString::number(obj.value(key).toDouble(), 'g', 6));
            ++applied;
        }
    }
    m_log->appendPlainText(QString("[Parametersatz importiert: %1 von %2 Werten - "
                                   "zum Übernehmen einzeln schreiben]")
                           .arg(applied).arg(CAL_VALUES));
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
        m_log->appendPlainText(QString("[%1 geschrieben - \"Im DFLASH speichern\" "
                                       "nicht vergessen]")
                               .arg(QString::fromUtf8(NVM_PARAMS[row].key)));
        readNvmParams();        // read back for confirmation
        return;
    }

    if (address >= XCP_CAL_ADDR + 4 && address < XCP_CAL_ADDR + 4 + quint32(CAL_VALUES) * 4) {
        const int row = int((address - (XCP_CAL_ADDR + 4)) / 4);
        m_log->appendPlainText(QString("[%1 geschrieben]")
                               .arg(QString::fromUtf8(CAL_PARAMS[row].key)));
        readCalibration();      // read back for confirmation
    }
}

void XcpPanel::onError(const QString &message)
{
    m_log->appendPlainText("[Fehler: " + message + "]");
}

void XcpPanel::pollTick()
{
    if (m_client->daqActive()) {
        // DAQ streams by itself; the timer only watches for silence
        if (m_lastDaqMs > 0 && m_timeBase.elapsed() - m_lastDaqMs > 1500) {
            m_log->appendPlainText("[DAQ-Strom abgerissen]");
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
        m_identLbl->setText("-");
        m_diagSummary->setText("Nicht verbunden");
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
