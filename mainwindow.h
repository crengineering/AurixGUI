#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QFutureWatcher>

// Forward declarations: only pointer types needed here, not full headers.
// Saves compile time; real includes are in the .cpp.
class QCheckBox;
class QComboBox;
class QPushButton;
class QPlainTextEdit;
class QTabWidget;
class QTimer;

// Result of one background port scan (worker thread, see scanPortsWorker).
struct PortScan
{
    QList<QSerialPortInfo> ports;
    QString aurixPort;   // empty = no responsive AURIX port found
};

class MainWindow : public QMainWindow
{
    Q_OBJECT   // Required macro for signals/slots. Triggers moc.

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void refreshPorts();                                   // Rescan available ports
    void toggleConnection();                               // Connect / Disconnect
    void readData();                                       // Read incoming data
    void handleError(QSerialPort::SerialPortError error);  // e.g. board unplugged
    void tryAutoConnect();                                 // Auto-connect / retry
    void onPortScanFinished();                             // Background scan done

private:
    void setConnectedState(bool connected);                // Enable/disable UI controls
    void startPortScan();                                  // Launch background scan

    QComboBox      *m_portBox          = nullptr;
    QComboBox      *m_baudBox          = nullptr;
    QPushButton    *m_connectBtn       = nullptr;
    QPushButton    *m_refreshBtn       = nullptr;
    QPushButton    *m_clearBtn         = nullptr;
    QCheckBox      *m_autoClearChk     = nullptr;
    QPlainTextEdit *m_output           = nullptr;
    QSerialPort    *m_serial           = nullptr;
    QTimer         *m_autoConnectTimer = nullptr;
    QTimer         *m_heartbeatTimer   = nullptr;   // 500 ms 'H' for diag bit 11
    QTabWidget     *m_tabs             = nullptr;   // main tabs (lamp icons)
    bool            m_autoConnect      = true;

    QFutureWatcher<PortScan> m_scanWatcher;         // background port scan
};

#endif // MAINWINDOW_H
