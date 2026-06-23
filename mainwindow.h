#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>

// Forward-Declarations: wir brauchen hier nur die Zeiger-Typen, nicht die
// vollen Header. Spart Kompilierzeit. Die echten Includes stehen im .cpp.
class QComboBox;
class QPushButton;
class QPlainTextEdit;
class QTimer;

class MainWindow : public QMainWindow
{
    Q_OBJECT   // Pflicht-Makro fuer alles mit Signals/Slots. Triggert moc.

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void refreshPorts();                                   // Port-Liste neu einlesen
    void toggleConnection();                               // Verbinden / Trennen
    void readData();                                       // Daten vom Port lesen
    void handleError(QSerialPort::SerialPortError error);  // z.B. Board abgezogen
    void tryAutoConnect();                                 // Auto-Verbinden / Retry

private:
    void setConnectedState(bool connected);                // UI aktiv/inaktiv schalten

    QComboBox      *m_portBox          = nullptr;
    QComboBox      *m_baudBox          = nullptr;
    QPushButton    *m_connectBtn       = nullptr;
    QPushButton    *m_refreshBtn       = nullptr;
    QPlainTextEdit *m_output           = nullptr;
    QSerialPort    *m_serial           = nullptr;
    QTimer         *m_autoConnectTimer = nullptr;
    bool            m_autoConnect      = true;
};

#endif // MAINWINDOW_H
