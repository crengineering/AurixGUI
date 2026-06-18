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

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("AURIX Serial Monitor");
    resize(700, 500);

    // 'this' als Parent: Qt loescht m_serial automatisch mit dem Fenster.
    // Das ersetzt das, was in Python der Garbage Collector gemacht hat.
    m_serial = new QSerialPort(this);

    // ---- Widgets erzeugen ----
    m_portBox    = new QComboBox;
    m_baudBox    = new QComboBox;
    m_refreshBtn = new QPushButton("Aktualisieren");
    m_connectBtn = new QPushButton("Verbinden");
    m_output     = new QPlainTextEdit;
    m_output->setReadOnly(true);

    // Gaengige Baudraten. 115200 ist typisch fuer ASCLIN/UART am AURIX.
    const QList<qint32> bauds = {9600, 19200, 38400, 57600, 115200, 230400, 921600};
    for (qint32 b : bauds)
        m_baudBox->addItem(QString::number(b), b);
    m_baudBox->setCurrentText("115200");

    // ---- Layout ----
    auto *topRow = new QHBoxLayout;
    topRow->addWidget(new QLabel("Port:"));
    topRow->addWidget(m_portBox, 1);
    topRow->addWidget(new QLabel("Baud:"));
    topRow->addWidget(m_baudBox);
    topRow->addWidget(m_refreshBtn);
    topRow->addWidget(m_connectBtn);

    auto *layout = new QVBoxLayout;
    layout->addLayout(topRow);
    layout->addWidget(m_output, 1);

    auto *central = new QWidget;
    central->setLayout(layout);
    setCentralWidget(central);

    // ---- Signals/Slots verbinden ----
    connect(m_refreshBtn, &QPushButton::clicked,    this, &MainWindow::refreshPorts);
    connect(m_connectBtn, &QPushButton::clicked,    this, &MainWindow::toggleConnection);
    connect(m_serial,     &QSerialPort::readyRead,  this, &MainWindow::readData);
    connect(m_serial,     &QSerialPort::errorOccurred, this, &MainWindow::handleError);

    refreshPorts();
}

void MainWindow::refreshPorts()
{
    m_portBox->clear();

    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : ports) {
        QString label = info.portName();              // z.B. "COM3"
        if (!info.description().isEmpty())
            label += " - " + info.description();      // z.B. "COM3 - USB Serial Port"
        // Anzeigetext + echter Portname als hinterlegte Daten.
        m_portBox->addItem(label, info.portName());
    }

    if (m_portBox->count() == 0)
        m_output->appendPlainText("[Kein serieller Port gefunden]");
}

void MainWindow::toggleConnection()
{
    // Ist der Port offen -> trennen.
    if (m_serial->isOpen()) {
        m_serial->close();
        setConnectedState(false);
        m_output->appendPlainText("[Getrennt]");
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

    // Phase 1: nur lesen. Fuer Phase 2 (Senden) auf QIODevice::ReadWrite aendern.
    if (m_serial->open(QIODevice::ReadOnly)) {
        setConnectedState(true);
        m_output->appendPlainText(
            QString("[Verbunden: %1 @ %2 Baud]")
                .arg(m_serial->portName())
                .arg(m_serial->baudRate()));
    } else {
        m_output->appendPlainText("[Oeffnen fehlgeschlagen: " + m_serial->errorString() + "]");
    }
}

void MainWindow::readData()
{
    // readyRead feuert, sobald Daten da sind. Es kann ein Teil-Paket sein,
    // also NICHT von vollstaendigen Zeilen ausgehen.
    const QByteArray data = m_serial->readAll();

    // insertPlainText statt appendPlainText: append wuerde bei jedem Chunk
    // eine neue Zeile erzwingen und den Datenstrom zerhacken.
    m_output->moveCursor(QTextCursor::End);
    m_output->insertPlainText(QString::fromUtf8(data));
    m_output->moveCursor(QTextCursor::End);
}

void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    // ResourceError = Port verschwunden (z.B. Board abgezogen / Kabel raus).
    if (error == QSerialPort::ResourceError) {
        m_output->appendPlainText("[Verbindungsfehler: " + m_serial->errorString() + "]");
        if (m_serial->isOpen())
            m_serial->close();
        setConnectedState(false);
    }
}

void MainWindow::setConnectedState(bool connected)
{
    m_connectBtn->setText(connected ? "Trennen" : "Verbinden");
    // Waehrend der Verbindung Port/Baud sperren - sonst inkonsistenter Zustand.
    m_portBox->setEnabled(!connected);
    m_baudBox->setEnabled(!connected);
    m_refreshBtn->setEnabled(!connected);
}
