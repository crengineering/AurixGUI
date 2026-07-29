#ifndef XCPCLIENT_H
#define XCPCLIENT_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QQueue>

class QTimer;

// Minimal XCP-on-Ethernet (UDP) master, counterpart to Xcp.c on the TC399.
// Transport framing: 2-byte length + 2-byte counter (little-endian) in front
// of every XCP packet. Requests are queued and sent one at a time; a QTimer
// guards against lost datagrams.
class XcpClient : public QObject
{
    Q_OBJECT

public:
    struct Measurements {
        quint8  verMajor   = 0;
        quint8  verMinor   = 0;
        quint8  verStep    = 0;
        quint32 tickMs     = 0;
        float   dieTempC   = 0.0f;  // PMS DTS (standby domain)
        float   dtscTempC  = 0.0f;  // SCU DTSC (core domain)
        float   vddCore    = 0.0f;  // 1.25 V rail [V]
        float   vddp3      = 0.0f;  // 3.3 V rail [V]
        float   vext       = 0.0f;  // 5 V board supply [V]
        quint32 diagStatus = 0;     // diagnostics bitmask (DIAGNOSTICS.md)
        float   baroPressPa = 0.0f; // BMP388 pressure [Pa]
        float   baroTempC   = 0.0f; // BMP388 temperature [degC]
        float   baroAltM    = 0.0f; // pressure altitude [m]
        bool    baroPresent = false;// BMP388 answered at init
        bool    valid      = false; // magic word matched
    };

    explicit XcpClient(QObject *parent = nullptr);

    void connectToSlave(const QString &host, quint16 port);
    void disconnectFromSlave();
    bool isConnected() const { return m_connected; }

    void pollMeasurements(quint32 address);           // cyclic Xcp_Data read
    void readMemory(quint32 address, quint8 len);     // -> memoryRead()
    void writeMemory(quint32 address, const QByteArray &data); // -> memoryWritten()

    // Configure and start the DAQ list (event channel 0, 100 ms on the
    // board): one ODT entry covering tick..diagStatus. Measurements then
    // arrive event-driven via measurementsReceived() without polling.
    void startDaq(quint32 entryAddress, quint8 entrySize);
    bool daqActive() const { return m_daqActive; }

signals:
    void connected(const QString &identString);
    void disconnected();
    void measurementsReceived(const XcpClient::Measurements &m);
    void memoryRead(quint32 address, const QByteArray &data);
    void memoryWritten(quint32 address);
    void daqStarted();
    void daqFailed();
    void errorOccurred(const QString &message);

private slots:
    void onReadyRead();
    void onTimeout();

private:
    enum class ReqType { Connect, GetId, UploadId, Poll, MemRead, MemWrite,
                         DaqCmd, DaqStart };
    struct Request {
        ReqType    type;
        QByteArray packet;
        quint32    addr = 0;
    };

    void enqueue(Request req);
    void sendNext();
    void handleResponse(const QByteArray &packet);
    void handleDaqFrame(const QByteArray &packet);
    void dropConnection(const QString &reason);

    QUdpSocket     *m_socket  = nullptr;
    QTimer         *m_timeout = nullptr;
    QHostAddress    m_host;
    quint16         m_port     = 5555;
    quint16         m_ctr      = 0;
    bool            m_connected = false;
    bool            m_busy      = false;
    Request         m_current;
    QQueue<Request> m_queue;
    quint32         m_identLen  = 0;
    bool            m_daqActive = false;
    quint8          m_verMajor  = 0;    // cached from the last poll; DAQ
    quint8          m_verMinor  = 0;    // frames do not carry the version
    quint8          m_verStep   = 0;
    bool            m_verValid  = false;
};

#endif // XCPCLIENT_H
