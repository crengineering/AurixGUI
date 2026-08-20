#ifndef XCPCLIENT_H
#define XCPCLIENT_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QQueue>
#include <QVector>
#include <QPair>

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
        float   accelX      = 0.0f; // MPU-6050 acceleration [g]
        float   accelY      = 0.0f;
        float   accelZ      = 0.0f;
        float   gyroX       = 0.0f; // MPU-6050 angular rate [deg/s]
        float   gyroY       = 0.0f;
        float   gyroZ       = 0.0f;
        float   imuTempC    = 0.0f; // MPU-6050 die temperature [degC]
        bool    imuPresent  = false;// MPU-6050 answered at init

        // attitude estimate (Ahrs.c) - flight_ctrl.h conventions
        quint8  ahrsState   = 0;    // 0 calibrating, 1 running, 2 no sensor
        bool    ahrsAccOk   = false;// |a| ~= 1 g, accel being trusted
        float   roll        = 0.0f; // phi   [rad]
        float   pitch       = 0.0f; // theta [rad]
        float   yaw         = 0.0f; // psi   [rad], drifts (gyro-only)
        float   rateP       = 0.0f; // p [rad/s]
        float   rateQ       = 0.0f; // q [rad/s]
        float   rateR       = 0.0f; // r [rad/s]
        float   biasX       = 0.0f; // measured gyro bias [deg/s]
        float   biasY       = 0.0f;
        float   biasZ       = 0.0f;

        // Peripherals whose only named field is the presence flag: the samples
        // themselves are A2L-described and read straight out of raw, so nothing
        // else about them needs a struct member here.
        bool    magPresent  = false;// MMC5983MA answered at init
        bool    gnssPresent = false;// NEO-M9N answered at init

        // per-core execution time (CoreStats.c)
        quint32 coreExecUs[6]   = {0,0,0,0,0,0};  // busy time per 100 ms [us]
        quint16 coreLoadPmil[6] = {0,0,0,0,0,0};  // per mille of the window
        quint16 coreAlive[6]    = {0,0,0,0,0,0};  // frozen => core hung

        // Ethernet (EthStats.c)
        quint32 ethBytesPerSec = 0;
        quint16 ethUtilPmil    = 0; // 0..1000
        quint16 ethLinkMbits   = 0; // 10 / 100 / 1000

        // Raw copy of Xcp_Data, so anything described in the A2L can be
        // decoded generically by (ECU_ADDRESS - XCP_DATA_ADDR) without the
        // client needing a named field for it. Empty until the first poll.
        QByteArray raw;

        bool    valid      = false; // magic word matched
    };

    explicit XcpClient(QObject *parent = nullptr);

    void connectToSlave(const QString &host, quint16 port);
    void disconnectFromSlave();
    bool isConnected() const { return m_connected; }

    // Cyclic read of the whole measurement block. blockSize comes from the A2L
    // (A2lModel::blockExtent), so the transport never needs to know which
    // peripherals exist -- it just moves N bytes in MAX_CTO-sized chunks.
    void pollMeasurements(quint32 address, int blockSize);
    void readMemory(quint32 address, quint8 len);     // -> memoryRead()
    void writeMemory(quint32 address, const QByteArray &data); // -> memoryWritten()

    // Configure and start the DAQ list (event channel 0, 100 ms on the board).
    // The block exceeds the 63-byte MAX_DTO, so it is split across
    // ceil(blockSize / 63) ODTs of consecutive bytes; ODT i covers
    // [i*63, i*63+len). Measurements then arrive event-driven via
    // measurementsReceived() without polling.
    //
    // Both the ODT count and their extents are derived from blockSize, so
    // adding a field to Xcp_Data and its A2L entry is enough -- there is no
    // per-peripheral branch to forget. (Forgetting one is exactly what made the
    // magnetometer read a constant 0 for a while: its ODT was allocated, the
    // frames arrived, and nothing stored them.)
    void startDaq(quint32 baseAddress, int blockSize);
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
    enum class ReqType { Connect, GetId, UploadId, PollChunk,
                         MemRead, MemWrite, DaqCmd, DaqStart };
    struct Request {
        ReqType    type;
        QByteArray packet;
        quint32    addr = 0;
        int        off  = 0;      // PollChunk: offset into the block
        int        len  = 0;      // PollChunk: bytes requested
        bool       last = false;  // PollChunk: emit after storing this one
    };

    void enqueue(Request req);
    void sendNext();
    void handleResponse(const QByteArray &packet);
    void handleDaqFrame(const QByteArray &packet);
    void parseImu(const char *d);   // fill m_accum IMU fields from a 32-byte block
    void parseAhrs(const char *d);  // fill m_accum attitude fields (40-byte block)
    void parseSys(const char *d);   // fill m_accum core/Ethernet stats (56-byte block)
    void storeRaw(int offset, const char *d, int len);  // mirror into m_accum.raw
    void dropConnection(const QString &reason);

    // Fill the named Measurements fields from the assembled m_accum.raw. Only a
    // convenience layer over raw, for the footer and the older tabs that use
    // C++ struct members; anything A2L-described is read out of raw directly and
    // needs nothing here.
    void parseNamedFields();

    // Byte extents of the ODT / SHORT_UPLOAD chunks the block is split into.
    QVector<QPair<int, int>> blockChunks() const;   // (offset, length)

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
    // Running measurement state assembled across the poll chunks / DAQ ODTs
    // before a single measurementsReceived() is emitted.
    Measurements    m_accum;
    quint32         m_blockBase = 0;    // Xcp_Data base address
    int             m_blockSize = 0;    // bytes to fetch, from the A2L
    int             m_lastOdt   = 0;    // highest ODT index in use
};

#endif // XCPCLIENT_H
