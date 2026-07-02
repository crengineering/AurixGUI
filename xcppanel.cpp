#include "xcppanel.h"

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
#include <cstring>

namespace {
// Fixed addresses, see Measurements.h / Diagnostics.h in the firmware.
constexpr quint32 XCP_DATA_ADDR = 0x70030000;
constexpr quint32 XCP_CAL_ADDR  = 0x70030100;
constexpr int     CAL_VALUES    = 15;   // floats after the magic word
constexpr int     POLL_MS       = 100;  // 10 Hz

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
    {31, "Kalibrierblock ungültig - Defaults geladen"},
};
constexpr int DIAG_ROWS = int(sizeof(DIAG_BITS) / sizeof(DIAG_BITS[0]));

// calibration block layout (offset after magic = index * 4)
struct CalParam { const char *name; const char *unit; };
constexpr CalParam CAL_PARAMS[CAL_VALUES] = {
    {"dtsMin   - DTS Untergrenze",        "°C"},
    {"dtsMax   - DTS Obergrenze",         "°C"},
    {"dtscMin  - DTSC Untergrenze",       "°C"},
    {"dtscMax  - DTSC Obergrenze",        "°C"},
    {"vddMin   - VDD Untergrenze",        "V"},
    {"vddMax   - VDD Obergrenze",         "V"},
    {"vddp3Min - VDDP3 Untergrenze",      "V"},
    {"vddp3Max - VDDP3 Obergrenze",       "V"},
    {"vextMin  - VEXT Untergrenze",       "V"},
    {"vextMax  - VEXT Obergrenze",        "V"},
    {"tempDeltaMax - max. Sensor-Delta",  "K"},
    {"debounceSec  - Entprellzeit",       "s"},
    {"fsVdd    - ADC-Endwert VDD",        "V"},
    {"fsVddp3  - ADC-Endwert VDDP3",      "V"},
    {"fsVext   - ADC-Endwert VEXT",       "V"},
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

    auto *subTabs = new QTabWidget;
    subTabs->addTab(buildLiveTab(), "Messwerte");
    subTabs->addTab(buildDiagTab(), "Diagnose");
    subTabs->addTab(buildCalTab(),  "Kalibrierung");

    auto *layout = new QVBoxLayout(this);
    layout->addLayout(topRow);
    layout->addWidget(subTabs, 1);

    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(POLL_MS);

    connect(m_connectBtn, &QPushButton::clicked, this, &XcpPanel::toggleConnection);
    connect(m_pollTimer,  &QTimer::timeout,      this, &XcpPanel::pollTick);
    connect(m_client, &XcpClient::connected,            this, &XcpPanel::onConnected);
    connect(m_client, &XcpClient::disconnected,         this, &XcpPanel::onDisconnected);
    connect(m_client, &XcpClient::measurementsReceived, this, &XcpPanel::onMeasurements);
    connect(m_client, &XcpClient::memoryRead,           this, &XcpPanel::onMemoryRead);
    connect(m_client, &XcpClient::memoryWritten,        this, &XcpPanel::onMemoryWritten);
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

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addWidget(m_diagSummary);
    layout->addWidget(m_diagWordLbl);
    layout->addWidget(m_diagTable, 1);
    layout->addWidget(new QLabel("Interpretation siehe DIAGNOSTICS.md im Firmware-Repository."));
    return tab;
}

QWidget *XcpPanel::buildCalTab()
{
    m_calTable = new QTableWidget(CAL_VALUES, 3);
    m_calTable->setHorizontalHeaderLabels({"Parameter", "Einheit", "Wert"});
    m_calTable->verticalHeader()->setVisible(false);
    for (int i = 0; i < CAL_VALUES; ++i) {
        auto *name = new QTableWidgetItem(QString::fromUtf8(CAL_PARAMS[i].name));
        name->setFlags(name->flags() & ~Qt::ItemIsEditable);
        auto *unit = new QTableWidgetItem(QString::fromUtf8(CAL_PARAMS[i].unit));
        unit->setFlags(unit->flags() & ~Qt::ItemIsEditable);
        m_calTable->setItem(i, 0, name);
        m_calTable->setItem(i, 1, unit);
        m_calTable->setItem(i, 2, new QTableWidgetItem("-"));   // editable
    }
    m_calTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    m_calReadBtn  = new QPushButton("Lesen");
    m_calWriteBtn = new QPushButton("Schreiben");
    m_calReadBtn->setEnabled(false);
    m_calWriteBtn->setEnabled(false);
    connect(m_calReadBtn,  &QPushButton::clicked, this, &XcpPanel::readCalibration);
    connect(m_calWriteBtn, &QPushButton::clicked, this, &XcpPanel::writeCalibration);

    auto *btnRow = new QHBoxLayout;
    btnRow->addWidget(m_calReadBtn);
    btnRow->addWidget(m_calWriteBtn);
    btnRow->addStretch(1);

    auto *tab    = new QWidget;
    auto *layout = new QVBoxLayout(tab);
    layout->addWidget(m_calTable, 1);
    layout->addLayout(btnRow);
    layout->addWidget(new QLabel("Werte liegen im RAM - nach einem Reset gelten wieder die Defaults."));
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
    m_pollTimer->start();
    readCalibration();          // populate the calibration tab right away
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
}

void XcpPanel::readCalibration()
{
    // 15 floats after the magic word (60 bytes, fits one SHORT_UPLOAD)
    m_client->readMemory(XCP_CAL_ADDR + 4, CAL_VALUES * 4);
}

void XcpPanel::onMemoryRead(quint32 address, const QByteArray &data)
{
    if (address != XCP_CAL_ADDR + 4 || data.size() < CAL_VALUES * 4)
        return;

    for (int i = 0; i < CAL_VALUES; ++i) {
        float f = 0.0f;
        std::memcpy(&f, data.constData() + i * 4, sizeof(f));
        m_calTable->item(i, 2)->setText(QString::number(double(f), 'g', 6));
    }
    m_log->appendPlainText("[Kalibrierwerte gelesen]");
}

void XcpPanel::writeCalibration()
{
    QByteArray block(CAL_VALUES * 4, '\0');
    for (int i = 0; i < CAL_VALUES; ++i) {
        bool  ok = false;
        float f  = m_calTable->item(i, 2)->text().replace(',', '.').toFloat(&ok);
        if (!ok) {
            m_log->appendPlainText(QString("[Ungültiger Wert in Zeile %1 - Abbruch]").arg(i + 1));
            return;
        }
        std::memcpy(block.data() + i * 4, &f, sizeof(f));
    }

    // SHORT_DOWNLOAD payload is limited to 56 bytes -> two chunks
    m_calWritesPending = 2;
    m_client->writeMemory(XCP_CAL_ADDR + 4,      block.left(40));
    m_client->writeMemory(XCP_CAL_ADDR + 4 + 40, block.mid(40));
}

void XcpPanel::onMemoryWritten(quint32 address)
{
    Q_UNUSED(address);
    if (m_calWritesPending > 0 && --m_calWritesPending == 0) {
        m_log->appendPlainText("[Kalibrierwerte geschrieben]");
        readCalibration();      // read back for confirmation
    }
}

void XcpPanel::onError(const QString &message)
{
    m_log->appendPlainText("[Fehler: " + message + "]");
}

void XcpPanel::pollTick()
{
    m_client->pollMeasurements(XCP_DATA_ADDR);
}

void XcpPanel::setConnectedState(bool connected)
{
    m_connectBtn->setText(connected ? "Disconnect" : "Connect");
    m_hostEdit->setEnabled(!connected);
    m_portBox->setEnabled(!connected);
    m_calReadBtn->setEnabled(connected);
    m_calWriteBtn->setEnabled(connected);

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
    }
}
