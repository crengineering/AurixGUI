#include "mainwindow.h"
#include "xcppanel.h"
#include "lampicon.h"

#include <QCheckBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include "systemfooter.h"
#include <QComboBox>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QTextCursor>
#include <QSerialPortInfo>
#include <QTimer>
#include <QtConcurrent/QtConcurrentRun>

namespace {
const QStringList kAurixKeywords = {"AURIX", "Infineon", "XMC", "DAS"};

// Runs in a worker thread. Enumerating COM ports and especially the first
// open() after a replug can block for seconds while Windows/the FTDI driver
// re-initialises the device — that must never happen on the GUI thread.
// The probe-open absorbs the slow first open, so the GUI thread afterwards
// only opens a port that is already known to respond quickly.
PortScan scanPortsWorker()
{
    PortScan scan;
    scan.ports = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &info : scan.ports) {
        for (const QString &kw : kAurixKeywords) {
            if (info.description().contains(kw, Qt::CaseInsensitive)) {
                QSerialPort probe;
                probe.setPortName(info.portName());
                if (probe.open(QIODevice::ReadOnly)) {
                    probe.close();
                    scan.aurixPort = info.portName();
                    return scan;
                }
                break;   // matched but busy/not ready yet — try next port
            }
        }
    }
    return scan;
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("AURIX Monitor");
    resize(700, 500);

    // 'this' as parent: Qt destroys m_serial automatically with the window.
    m_serial = new QSerialPort(this);

    // ---- Widgets ----
    m_portBox    = new QComboBox;
    m_baudBox    = new QComboBox;
    m_refreshBtn = new QPushButton("Refresh");
    m_connectBtn = new QPushButton("Connect");
    m_clearBtn     = new QPushButton("Clear");
    m_autoClearChk = new QCheckBox("Auto Clear");
    m_output     = new QPlainTextEdit;
    m_output->setReadOnly(true);

    // Common baud rates. 115200 is typical for ASCLIN/UART on AURIX.
    const QList<qint32> bauds = {9600, 19200, 38400, 57600, 115200, 230400, 921600};
    for (qint32 b : bauds)
        m_baudBox->addItem(QString::number(b), b);
    m_baudBox->setCurrentText("115200");

    // ---- Layout ----
    auto *topRow = new QHBoxLayout;
    topRow->addWidget(new QLabel("Port:"));
    topRow->addWidget(m_portBox, 1);
    topRow->addWidget(new QLabel("Baud rate:"));
    topRow->addWidget(m_baudBox);
    topRow->addWidget(m_refreshBtn);
    topRow->addWidget(m_connectBtn);

    auto *clearGroup = new QVBoxLayout;
    clearGroup->setSpacing(2);
    clearGroup->addWidget(m_clearBtn);
    clearGroup->addWidget(m_autoClearChk);
    topRow->addLayout(clearGroup);

    auto *layout = new QVBoxLayout;
    layout->addLayout(topRow);
    layout->addWidget(m_output, 1);

    auto *uartTab = new QWidget;
    uartTab->setLayout(layout);

    // Tabs: UART monitor (existing) + Ethernet/XCP live view. The lamp icons
    // show the connection state (green = connected, red = not connected)
    // without having to open the tab.
    auto *xcpPanel = new XcpPanel;
    m_tabs = new QTabWidget;
    m_tabs->addTab(uartTab, lampIcon(LampColor::Red), "UART");
    m_tabs->addTab(xcpPanel, lampIcon(LampColor::Red), "Ethernet");
    // Tabs on top, permanent status strip underneath. The footer is deliberately
    // outside the tab widget so core load and link utilisation stay visible no
    // matter which tab is in front.
    m_footer = new SystemFooter;

    auto *centralWrap = new QWidget;
    auto *centralCol  = new QVBoxLayout(centralWrap);
    centralCol->setContentsMargins(0, 0, 0, 0);
    centralCol->setSpacing(0);
    centralCol->addWidget(m_tabs, 1);
    centralCol->addWidget(m_footer, 0);
    setCentralWidget(centralWrap);

    connect(xcpPanel, &XcpPanel::connectionChanged, this, [this](bool connected) {
        m_tabs->setTabIcon(1, lampIcon(connected ? LampColor::Green
                                                 : LampColor::Red));
        m_footer->setConnected(connected);
    });
    connect(xcpPanel, &XcpPanel::measurementsUpdated,
            m_footer, &SystemFooter::update);

    // ---- Signals/Slots ----
    connect(m_refreshBtn, &QPushButton::clicked,    this, &MainWindow::refreshPorts);
    connect(m_connectBtn, &QPushButton::clicked,    this, &MainWindow::toggleConnection);
    connect(m_clearBtn,   &QPushButton::clicked,    m_output, &QPlainTextEdit::clear);
    connect(m_serial,     &QSerialPort::readyRead,  this, &MainWindow::readData);
    connect(m_serial,     &QSerialPort::errorOccurred, this, &MainWindow::handleError);

    m_autoConnectTimer = new QTimer(this);
    m_autoConnectTimer->setInterval(2000);
    connect(m_autoConnectTimer, &QTimer::timeout, this, &MainWindow::tryAutoConnect);
    connect(&m_scanWatcher, &QFutureWatcher<PortScan>::finished,
            this, &MainWindow::onPortScanFinished);

    // Heartbeat: the firmware clears diag bit 11 (UART disconnected) as long
    // as it keeps receiving bytes; > 2 s silence sets the bit. 500 ms gives
    // a 4x margin. Byte value is irrelevant, any RX activity counts.
    m_heartbeatTimer = new QTimer(this);
    m_heartbeatTimer->setInterval(500);
    connect(m_heartbeatTimer, &QTimer::timeout, this, [this]() {
        if (m_serial->isOpen())
            m_serial->write("H", 1);
    });

    tryAutoConnect();
}

void MainWindow::refreshPorts()
{
    startPortScan();
}

void MainWindow::startPortScan()
{
    if (m_scanWatcher.isRunning())
        return;                     // one scan at a time is plenty
    m_scanWatcher.setFuture(QtConcurrent::run(scanPortsWorker));
}

void MainWindow::onPortScanFinished()
{
    const PortScan scan = m_scanWatcher.result();

    // While connected the combo is locked anyway — don't clobber it.
    if (!m_serial->isOpen()) {
        m_portBox->clear();
        for (const QSerialPortInfo &info : scan.ports) {
            QString label = info.portName();              // e.g. "COM3"
            if (!info.description().isEmpty())
                label += " - " + info.description();      // e.g. "COM3 - USB Serial Port"
            // Display text + actual port name stored as item data.
            m_portBox->addItem(label, info.portName());
        }
        if (m_portBox->count() == 0)
            m_output->appendPlainText("[No serial port found]");
    }

    if (m_serial->isOpen() || !m_autoConnect)
        return;

    if (!scan.aurixPort.isEmpty()) {
        const int idx = m_portBox->findData(scan.aurixPort);
        if (idx >= 0) {
            m_portBox->setCurrentIndex(idx);
            toggleConnection();
        }
    }

    if (m_serial->isOpen()) {
        m_autoConnectTimer->stop();
        return;
    }

    // No responsive AURIX port yet — report once, then retry silently.
    if (!m_autoConnectTimer->isActive()) {
        m_output->appendPlainText("[Searching for AURIX port... (AURIX / Infineon / XMC / DAS)]");
        m_autoConnectTimer->start();
    }
}

void MainWindow::toggleConnection()
{
    // Port is open -> disconnect.
    if (m_serial->isOpen()) {
        m_autoConnect = false;
        m_autoConnectTimer->stop();
        m_serial->close();
        setConnectedState(false);
        m_output->appendPlainText("[Disconnected]");
        return;
    }

    if (m_portBox->count() == 0)
        return;

    m_serial->setPortName(m_portBox->currentData().toString());
    m_serial->setBaudRate(m_baudBox->currentData().toInt());
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);

    // ReadWrite: reading the debug output plus sending the heartbeat byte.
    if (m_serial->open(QIODevice::ReadWrite)) {
        setConnectedState(true);
        m_output->appendPlainText(
            QString("[Connected: %1 @ %2 Baud]")
                .arg(m_serial->portName())
                .arg(m_serial->baudRate()));
    } else {
        m_output->appendPlainText("[Failed to open: " + m_serial->errorString() + "]");
    }
}

void MainWindow::readData()
{
    // readyRead fires as soon as data arrives; it may be a partial packet,
    // so do NOT assume complete lines.
    const QByteArray data = m_serial->readAll();

    // "CPU0 started" is the first UART message after a fresh flash — use it
    // as a reliable reset marker to clear old output automatically.
    if (m_autoClearChk->isChecked() && data.contains("CPU1 started"))
        m_output->clear();

    // insertPlainText instead of appendPlainText: append would force a new line
    // on every chunk and break up the data stream.
    m_output->moveCursor(QTextCursor::End);
    m_output->insertPlainText(QString::fromUtf8(data));
    m_output->moveCursor(QTextCursor::End);
}

void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    // ResourceError = port disappeared (e.g. board unplugged / cable pulled).
    if (error == QSerialPort::ResourceError) {
        m_output->appendPlainText("[Connection error: " + m_serial->errorString() + "]");
        if (m_serial->isOpen())
            m_serial->close();
        setConnectedState(false);
        m_autoConnect = true;
        tryAutoConnect();
    }
}

void MainWindow::setConnectedState(bool connected)
{
    m_connectBtn->setText(connected ? "Disconnect" : "Connect");
    // Lock port/baud controls while connected to prevent inconsistent state.
    m_portBox->setEnabled(!connected);
    m_baudBox->setEnabled(!connected);
    m_refreshBtn->setEnabled(!connected);

    m_tabs->setTabIcon(0, lampIcon(connected ? LampColor::Green
                                             : LampColor::Red));
    if (connected)
        m_heartbeatTimer->start();
    else
        m_heartbeatTimer->stop();
}

void MainWindow::tryAutoConnect()
{
    if (m_serial->isOpen())
        return;

    // All blocking work happens in the worker; onPortScanFinished connects.
    startPortScan();
}
