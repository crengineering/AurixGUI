#include "mainwindow.h"

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

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("AURIX Serial Monitor");
    resize(700, 500);

    // 'this' as parent: Qt destroys m_serial automatically with the window.
    m_serial = new QSerialPort(this);

    // ---- Widgets ----
    m_portBox    = new QComboBox;
    m_baudBox    = new QComboBox;
    m_refreshBtn = new QPushButton("Refresh");
    m_connectBtn = new QPushButton("Connect");
    m_clearBtn   = new QPushButton("Clear");
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
    topRow->addWidget(m_clearBtn);

    auto *layout = new QVBoxLayout;
    layout->addLayout(topRow);
    layout->addWidget(m_output, 1);

    auto *central = new QWidget;
    central->setLayout(layout);
    setCentralWidget(central);

    // ---- Signals/Slots ----
    connect(m_refreshBtn, &QPushButton::clicked,    this, &MainWindow::refreshPorts);
    connect(m_connectBtn, &QPushButton::clicked,    this, &MainWindow::toggleConnection);
    connect(m_clearBtn,   &QPushButton::clicked,    m_output, &QPlainTextEdit::clear);
    connect(m_serial,     &QSerialPort::readyRead,  this, &MainWindow::readData);
    connect(m_serial,     &QSerialPort::errorOccurred, this, &MainWindow::handleError);

    m_autoConnectTimer = new QTimer(this);
    m_autoConnectTimer->setInterval(2000);
    connect(m_autoConnectTimer, &QTimer::timeout, this, &MainWindow::tryAutoConnect);
    tryAutoConnect();
}

void MainWindow::refreshPorts()
{
    m_portBox->clear();

    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : ports) {
        QString label = info.portName();              // e.g. "COM3"
        if (!info.description().isEmpty())
            label += " - " + info.description();      // e.g. "COM3 - USB Serial Port"
        // Display text + actual port name stored as item data.
        m_portBox->addItem(label, info.portName());
    }

    if (m_portBox->count() == 0)
        m_output->appendPlainText("[No serial port found]");
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

    // Phase 1: read-only. Change to QIODevice::ReadWrite for Phase 2 (sending).
    if (m_serial->open(QIODevice::ReadOnly)) {
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
}

void MainWindow::tryAutoConnect()
{
    if (m_serial->isOpen())
        return;

    refreshPorts();

    static const QStringList keywords = {"AURIX", "Infineon", "XMC", "DAS"};
    const auto ports = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &info : ports) {
        for (const QString &kw : keywords) {
            if (info.description().contains(kw, Qt::CaseInsensitive)) {
                const int idx = m_portBox->findData(info.portName());
                if (idx >= 0) {
                    m_portBox->setCurrentIndex(idx);
                    toggleConnection();
                }
                if (m_serial->isOpen()) {
                    m_autoConnectTimer->stop();
                    return;
                }
                break;
            }
        }
    }

    // No matching port found or open failed — report once, then retry silently.
    if (!m_autoConnectTimer->isActive()) {
        m_output->appendPlainText("[Searching for AURIX port... (AURIX / Infineon / XMC / DAS)]");
        m_autoConnectTimer->start();
    }
}
