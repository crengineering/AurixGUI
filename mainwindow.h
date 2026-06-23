#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>

// Forward declarations: only pointer types needed here, not full headers.
// Saves compile time; real includes are in the .cpp.
class QComboBox;
class QPushButton;
class QPlainTextEdit;
class QTimer;

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

private:
    void setConnectedState(bool connected);                // Enable/disable UI controls

    QComboBox      *m_portBox          = nullptr;
    QComboBox      *m_baudBox          = nullptr;
    QPushButton    *m_connectBtn       = nullptr;
    QPushButton    *m_refreshBtn       = nullptr;
    QPushButton    *m_clearBtn         = nullptr;
    QPlainTextEdit *m_output           = nullptr;
    QSerialPort    *m_serial           = nullptr;
    QTimer         *m_autoConnectTimer = nullptr;
    bool            m_autoConnect      = true;
};

#endif // MAINWINDOW_H
