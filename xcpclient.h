#ifndef XCPCLIENT_H
#define XCPCLIENT_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>

class QTimer;

// Minimal XCP-on-Ethernet (UDP) master, counterpart to Xcp.c on the TC399.
// Transport framing: 2-byte length + 2-byte counter (little-endian) in front
// of every XCP packet. One request in flight at a time; a QTimer guards
// against lost datagrams.
class XcpClient : public QObject
{
    Q_OBJECT

public:
    struct Measurements {
        quint8  verMajor  = 0;
        quint8  verMinor  = 0;
        quint8  verStep   = 0;
        quint32 tickMs    = 0;
        float   dieTempC  = 0.0f;   // PMS DTS (standby domain)
        float   dtscTempC = 0.0f;   // SCU DTSC (core domain)
        float   vddCore   = 0.0f;   // 1.25 V rail [V]
        float   vddp3     = 0.0f;   // 3.3 V rail [V]
        float   vext      = 0.0f;   // 5 V board supply [V]
        bool    valid     = false;  // magic word matched
    };

    explicit XcpClient(QObject *parent = nullptr);

    void connectToSlave(const QString &host, quint16 port);
    void disconnectFromSlave();
    bool isConnected() const { return m_state == State::Connected; }

    // Cyclic read of the Xcp_Data struct (SHORT_UPLOAD, 16 bytes).
    void pollMeasurements(quint32 address);

signals:
    void connected(const QString &identString);
    void disconnected();
    void measurementsReceived(const XcpClient::Measurements &m);
    void errorOccurred(const QString &message);

private slots:
    void onReadyRead();
    void onTimeout();

private:
    enum class State { Idle, Connecting, GettingId, UploadingId, Connected };

    void sendCommand(const QByteArray &packet);
    void handleResponse(const QByteArray &packet);

    QUdpSocket  *m_socket  = nullptr;
    QTimer      *m_timeout = nullptr;
    QHostAddress m_host;
    quint16      m_port    = 5555;
    quint16      m_ctr     = 0;
    State        m_state   = State::Idle;
    quint32      m_identLen = 0;
    bool         m_pollPending = false;
};

#endif // XCPCLIENT_H
