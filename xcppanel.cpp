#include "xcppanel.h"

#include <QLineEdit>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QPlainTextEdit>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDateTime>

namespace {
// Fixed address of the Xcp_Data struct, see Measurements.h in the firmware.
constexpr quint32 XCP_DATA_ADDR = 0x70030000;
constexpr int     POLL_MS       = 100;   // 10 Hz
}

XcpPanel::XcpPanel(QWidget *parent)
    : QWidget(parent)
{
    m_client = new XcpClient(this);

    // ---- Connection row ----
    m_hostEdit = new QLineEdit("192.168.0.10");
    m_portBox  = new QSpinBox;
    m_portBox->setRange(1, 65535);
    m_portBox->setValue(5555);
    m_connectBtn = new QPushButton("Connect");

    auto *topRow = new QHBoxLayout;
    topRow->addWidget(new QLabel("IP:"));
    topRow->addWidget(m_hostEdit, 1);
    topRow->addWidget(new QLabel("Port:"));
    topRow->addWidget(m_portBox);
    topRow->addWidget(m_connectBtn);

    // ---- Live values ----
    m_identLbl   = new QLabel("-");
    m_versionLbl = new QLabel("-");
    m_uptimeLbl  = new QLabel("-");
    m_tempLbl    = new QLabel("-");

    QFont bigFont = m_tempLbl->font();
    bigFont.setPointSize(bigFont.pointSize() * 2);
    bigFont.setBold(true);
    m_tempLbl->setFont(bigFont);

    auto *form = new QFormLayout;
    form->addRow("Board:",            m_identLbl);
    form->addRow("Software-Version:", m_versionLbl);
    form->addRow("Uptime:",           m_uptimeLbl);
    form->addRow("Die-Temperatur:",   m_tempLbl);

    // ---- Log ----
    m_log = new QPlainTextEdit;
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(500);

    auto *layout = new QVBoxLayout(this);
    layout->addLayout(topRow);
    layout->addLayout(form);
    layout->addWidget(m_log, 1);

    // ---- Poll timer (runs only while connected) ----
    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(POLL_MS);

    connect(m_connectBtn, &QPushButton::clicked, this, &XcpPanel::toggleConnection);
    connect(m_pollTimer,  &QTimer::timeout,      this, &XcpPanel::pollTick);
    connect(m_client, &XcpClient::connected,            this, &XcpPanel::onConnected);
    connect(m_client, &XcpClient::disconnected,         this, &XcpPanel::onDisconnected);
    connect(m_client, &XcpClient::measurementsReceived, this, &XcpPanel::onMeasurements);
    connect(m_client, &XcpClient::errorOccurred,        this, &XcpPanel::onError);
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

    if (!connected) {
        m_versionLbl->setText("-");
        m_uptimeLbl->setText("-");
        m_tempLbl->setText("-");
        m_identLbl->setText("-");
    }
}
