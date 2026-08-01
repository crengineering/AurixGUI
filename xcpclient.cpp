#include "xcpclient.h"

#include <QTimer>
#include <QtEndian>
#include <cstring>

namespace {
constexpr quint8 CMD_CONNECT        = 0xFF;
constexpr quint8 CMD_DISCONNECT     = 0xFE;
constexpr quint8 CMD_GET_ID         = 0xFA;
constexpr quint8 CMD_UPLOAD         = 0xF5;
constexpr quint8 CMD_SHORT_UPLOAD   = 0xF4;
constexpr quint8 CMD_SHORT_DOWNLOAD = 0xED;
constexpr quint8 CMD_SET_DAQ_PTR    = 0xE2;
constexpr quint8 CMD_WRITE_DAQ      = 0xE1;
constexpr quint8 CMD_SET_DAQ_MODE   = 0xE0;
constexpr quint8 CMD_START_STOP_DAQ = 0xDE;
constexpr quint8 CMD_FREE_DAQ       = 0xD6;
constexpr quint8 CMD_ALLOC_DAQ      = 0xD5;
constexpr quint8 CMD_ALLOC_ODT      = 0xD4;
constexpr quint8 CMD_ALLOC_ODT_ENTRY = 0xD3;

constexpr quint8 PID_RES = 0xFF;
constexpr quint8 PID_ERR = 0xFE;

constexpr quint32 XCP_DATA_MAGIC = 0x41555258;  // must match Measurements.h
constexpr int     XCP_DATA_SIZE  = 52;          // core block: magic..baroAlt
constexpr int     XCP_IMU_OFFSET = 52;          // IMU sub-block starts at 0x34
constexpr int     XCP_IMU_SIZE   = 32;          // imuPresent(+pad) + 7 floats
constexpr int     XCP_AHRS_OFFSET = 84;         // attitude sub-block at 0x54
constexpr int     XCP_AHRS_SIZE   = 40;         // state/accOk(+pad) + 9 floats
constexpr int     XCP_SYS_OFFSET  = 124;        // core + Ethernet stats at 0x7C
constexpr int     XCP_SYS_SIZE    = 56;         // 6*u32 + 6*u16 + 6*u16 + u32 + 2*u16
constexpr int     XCP_MAG_OFFSET  = 180;        // magnetometer sub-block at 0xB4
constexpr int     XCP_MAG_SIZE    = 24;         // magPresent(+pad) + 5 floats
constexpr int     XCP_BLOCK_SIZE  = 204;        // full Xcp_Data (Measurements.h)
                                                // grew 180 -> 204 with the
                                                // MMC5983MA block in fw v1.15.0
constexpr int     TIMEOUT_MS     = 500;
}

XcpClient::XcpClient(QObject *parent)
    : QObject(parent)
{
    m_socket = new QUdpSocket(this);
    connect(m_socket, &QUdpSocket::readyRead, this, &XcpClient::onReadyRead);

    m_timeout = new QTimer(this);
    m_timeout->setSingleShot(true);
    m_timeout->setInterval(TIMEOUT_MS);
    connect(m_timeout, &QTimer::timeout, this, &XcpClient::onTimeout);
}

void XcpClient::connectToSlave(const QString &host, quint16 port)
{
    if (m_connected || m_busy)
        disconnectFromSlave();

    m_host = QHostAddress(host);
    m_port = port;
    m_ctr  = 0;
    m_accum = Measurements{};   // clear stale state from a previous session

    if (m_host.isNull()) {
        emit errorOccurred(QString("Ungueltige IP-Adresse: %1").arg(host));
        return;
    }

    QByteArray cmd(2, '\0');
    cmd[0] = char(CMD_CONNECT);
    enqueue({ReqType::Connect, cmd, 0});
}

void XcpClient::disconnectFromSlave()
{
    if (m_connected) {
        // fire and forget, no response tracking needed
        QByteArray frame(4, '\0');
        qToLittleEndian<quint16>(1, frame.data());
        qToLittleEndian<quint16>(m_ctr++, frame.data() + 2);
        frame += char(CMD_DISCONNECT);
        m_socket->writeDatagram(frame, m_host, m_port);
    }

    m_timeout->stop();
    m_queue.clear();
    m_busy      = false;
    m_connected = false;
    m_daqActive = false;
    m_verValid  = false;
    emit disconnected();
}

void XcpClient::pollMeasurements(quint32 address)
{
    if (!m_connected)
        return;

    // never let polls pile up behind a slow link
    for (const Request &r : m_queue)
        if (r.type == ReqType::Poll)
            return;
    if (m_busy && m_current.type == ReqType::Poll)
        return;

    // The block is 84 bytes > MAX_CTO(64), so read it in two SHORT_UPLOADs:
    // the 52-byte core, then the 32-byte IMU sub-block. The pair fills m_accum
    // and the IMU response emits the merged measurementsReceived().
    QByteArray core(8, '\0');
    core[0] = char(CMD_SHORT_UPLOAD);
    core[1] = char(XCP_DATA_SIZE);
    qToLittleEndian<quint32>(address, core.data() + 4);
    enqueue({ReqType::Poll, core, address});

    QByteArray imu(8, '\0');
    imu[0] = char(CMD_SHORT_UPLOAD);
    imu[1] = char(XCP_IMU_SIZE);
    qToLittleEndian<quint32>(address + XCP_IMU_OFFSET, imu.data() + 4);
    enqueue({ReqType::PollImu, imu, address + XCP_IMU_OFFSET});
}

void XcpClient::readMemory(quint32 address, quint8 len)
{
    if (!m_connected)
        return;

    QByteArray cmd(8, '\0');
    cmd[0] = char(CMD_SHORT_UPLOAD);
    cmd[1] = char(len);
    qToLittleEndian<quint32>(address, cmd.data() + 4);
    enqueue({ReqType::MemRead, cmd, address});
}

void XcpClient::writeMemory(quint32 address, const QByteArray &data)
{
    if (!m_connected || data.isEmpty() || data.size() > 56)
        return;

    QByteArray cmd(8, '\0');
    cmd[0] = char(CMD_SHORT_DOWNLOAD);
    cmd[1] = char(quint8(data.size()));
    qToLittleEndian<quint32>(address, cmd.data() + 4);
    cmd += data;
    enqueue({ReqType::MemWrite, cmd, address});
}

void XcpClient::startDaq(quint32 baseAddress)
{
    if (!m_connected)
        return;

    auto simple = [this](std::initializer_list<quint8> bytes) {
        QByteArray p;
        for (quint8 b : bytes)
            p.append(char(b));
        enqueue({ReqType::DaqCmd, p, 0});
    };
    // one WRITE_DAQ against the current DAQ pointer (odt/entry set beforehand)
    auto writeDaq = [this](quint8 size, quint32 addr) {
        QByteArray wr(8, '\0');
        wr[0] = char(CMD_WRITE_DAQ);
        wr[1] = char(0xFF);                                     // no bit offset
        wr[2] = char(size);
        qToLittleEndian<quint32>(addr, wr.data() + 4);
        enqueue({ReqType::DaqCmd, wr, 0});
    };

    // Four ODTs, one per sub-block. Every field the GUI shows must live in the
    // DAQ stream: after startDaq() the client stops polling, so anything not in
    // an ODT simply never updates (the core-load footer read 0 % until ODT2/ODT3
    // were added). Each ODT carries at most XCP_DAQ_MAX_ODT_DATA = 63 bytes and
    // the firmware allows XCP_DAQ_MAX_ODTS = 4, so 44/32/40/56 fits exactly.
    simple({CMD_FREE_DAQ});
    simple({CMD_ALLOC_DAQ, 0x00, 0x01, 0x00});                   // 1 list
    simple({CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x05});            // 5 ODTs
    simple({CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x01});// ODT0: 1 entry
    simple({CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x01, 0x01});// ODT1: 1 entry
    simple({CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x02, 0x01});// ODT2: 1 entry
    simple({CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x03, 0x01});// ODT3: 1 entry
    simple({CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x04, 0x01});// ODT4: 1 entry

    // ODT0 = core+baro (tick..baroAlt), 44 bytes at base+8
    simple({CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00});
    writeDaq(XCP_DATA_SIZE - 8, baseAddress + 8);
    // ODT1 = IMU sub-block, 32 bytes at base+52
    simple({CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x01, 0x00});
    writeDaq(XCP_IMU_SIZE, baseAddress + XCP_IMU_OFFSET);
    // ODT2 = attitude estimate, 40 bytes at base+84
    simple({CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x02, 0x00});
    writeDaq(XCP_AHRS_SIZE, baseAddress + XCP_AHRS_OFFSET);
    // ODT3 = core load + Ethernet, 56 bytes at base+124
    simple({CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x03, 0x00});
    writeDaq(XCP_SYS_SIZE, baseAddress + XCP_SYS_OFFSET);
    // ODT4 = magnetometer, 24 bytes at base+180 (MMC5983MA, fw >= v1.15.0)
    simple({CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x04, 0x00});
    writeDaq(XCP_MAG_SIZE, baseAddress + XCP_MAG_OFFSET);

    simple({CMD_SET_DAQ_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00});
    enqueue({ReqType::DaqStart, QByteArray("\xDE\x01\x00\x00", 4), 0});
}

void XcpClient::enqueue(Request req)
{
    m_queue.enqueue(std::move(req));
    if (!m_busy)
        sendNext();
}

void XcpClient::sendNext()
{
    if (m_queue.isEmpty()) {
        m_busy = false;
        return;
    }

    m_current = m_queue.dequeue();
    m_busy    = true;

    QByteArray frame(4, '\0');
    qToLittleEndian<quint16>(quint16(m_current.packet.size()), frame.data());
    qToLittleEndian<quint16>(m_ctr++, frame.data() + 2);
    frame += m_current.packet;

    m_socket->writeDatagram(frame, m_host, m_port);
    m_timeout->start();
}

void XcpClient::onReadyRead()
{
    while (m_socket->hasPendingDatagrams()) {
        QByteArray dgram(int(m_socket->pendingDatagramSize()), '\0');
        m_socket->readDatagram(dgram.data(), dgram.size());

        if (dgram.size() < 5)
            continue;                            // header + PID minimum

        const quint16 len = qFromLittleEndian<quint16>(dgram.constData());
        if (4 + len > dgram.size())
            continue;                            // malformed

        const QByteArray packet = dgram.mid(4, len);
        const quint8 pid = quint8(packet[0]);

        if (pid < 0xFC) {                        // DTO: DAQ data (PID = ODT nr)
            handleDaqFrame(packet);
            continue;
        }
        handleResponse(packet);
    }
}

void XcpClient::parseImu(const char *d)
{
    // 32-byte IMU sub-block: imuPresent u8 (+3 pad), then 7 floats.
    auto readF = [d](int off) {
        float f = 0.0f;
        std::memcpy(&f, d + off, sizeof(f));
        return f;
    };
    m_accum.imuPresent = (quint8(d[0]) != 0);
    m_accum.accelX   = readF(4);
    m_accum.accelY   = readF(8);
    m_accum.accelZ   = readF(12);
    m_accum.gyroX    = readF(16);
    m_accum.gyroY    = readF(20);
    m_accum.gyroZ    = readF(24);
    m_accum.imuTempC = readF(28);
}

void XcpClient::storeRaw(int offset, const char *d, int len)
{
    if (m_accum.raw.size() != XCP_BLOCK_SIZE)
        m_accum.raw = QByteArray(XCP_BLOCK_SIZE, char(0));
    if (offset >= 0 && len > 0 && offset + len <= XCP_BLOCK_SIZE)
        std::memcpy(m_accum.raw.data() + offset, d, size_t(len));
}

void XcpClient::parseAhrs(const char *d)
{
    // 40-byte attitude sub-block: state u8, accOk u8 (+2 pad), then 9 floats.
    auto readF = [d](int off) {
        float f = 0.0f;
        std::memcpy(&f, d + off, sizeof(f));
        return f;
    };
    m_accum.ahrsState = quint8(d[0]);
    m_accum.ahrsAccOk = (quint8(d[1]) != 0);
    m_accum.roll   = readF(4);
    m_accum.pitch  = readF(8);
    m_accum.yaw    = readF(12);
    m_accum.rateP  = readF(16);
    m_accum.rateQ  = readF(20);
    m_accum.rateR  = readF(24);
    m_accum.biasX  = readF(28);
    m_accum.biasY  = readF(32);
    m_accum.biasZ  = readF(36);
}

void XcpClient::parseSys(const char *d)
{
    // 56-byte system block: 6x u32 exec time, 6x u16 load, 6x u16 alive,
    // then u32 bytes/s, u16 util per mille, u16 link Mbit/s.
    for (int i = 0; i < 6; ++i)
        m_accum.coreExecUs[i] = qFromLittleEndian<quint32>(d + i * 4);
    for (int i = 0; i < 6; ++i)
        m_accum.coreLoadPmil[i] = qFromLittleEndian<quint16>(d + 24 + i * 2);
    for (int i = 0; i < 6; ++i)
        m_accum.coreAlive[i] = qFromLittleEndian<quint16>(d + 36 + i * 2);
    m_accum.ethBytesPerSec = qFromLittleEndian<quint32>(d + 48);
    m_accum.ethUtilPmil    = qFromLittleEndian<quint16>(d + 52);
    m_accum.ethLinkMbits   = qFromLittleEndian<quint16>(d + 54);
}

void XcpClient::handleDaqFrame(const QByteArray &packet)
{
    if (packet.isEmpty())
        return;

    const quint8 pid = quint8(packet[0]);
    const char  *d   = packet.constData() + 1;

    if (pid == 0) {
        // ODT0: tick u32, dts f, dtsc f, vdd f, vddp3 f, vext f, raw codes u32
        // (byte 27 = baroPresent), diagStatus u32, baroPress f, baroTemp f,
        // baroAlt f  (= Xcp_Data + 8, 44 bytes). Emits with the last IMU values
        // held in m_accum from the ODT1 frame.
        if (packet.size() < 1 + 44)
            return;
        auto readF = [d](int off) {
            float f = 0.0f;
            std::memcpy(&f, d + off, sizeof(f));
            return f;
        };
        m_accum.valid    = m_verValid;
        m_accum.verMajor = m_verMajor;
        m_accum.verMinor = m_verMinor;
        m_accum.verStep  = m_verStep;
        m_accum.tickMs   = qFromLittleEndian<quint32>(d);
        m_accum.dieTempC    = readF(4);
        m_accum.dtscTempC   = readF(8);
        m_accum.vddCore     = readF(12);
        m_accum.vddp3       = readF(16);
        m_accum.vext        = readF(20);
        m_accum.baroPresent = (quint8(d[27]) != 0);
        m_accum.diagStatus  = qFromLittleEndian<quint32>(d + 28);
        m_accum.baroPressPa = readF(32);
        m_accum.baroTempC   = readF(36);
        m_accum.baroAltM    = readF(40);
        storeRaw(8, d, 44);
        emit measurementsReceived(m_accum);
    } else if (pid == 1) {
        // ODT1: the 32-byte IMU sub-block. Stored into m_accum; the next ODT0
        // frame carries it out (both fire every 100 ms, so <=1 frame of lag).
        if (packet.size() < 1 + XCP_IMU_SIZE)
            return;
        parseImu(d);
        storeRaw(XCP_IMU_OFFSET, d, XCP_IMU_SIZE);
    } else if (pid == 2) {
        // ODT2: attitude estimate. Same store-and-carry pattern as ODT1.
        if (packet.size() < 1 + XCP_AHRS_SIZE)
            return;
        parseAhrs(d);
        storeRaw(XCP_AHRS_OFFSET, d, XCP_AHRS_SIZE);
    } else if (pid == 3) {
        // ODT3: per-core load + Ethernet, feeding the permanent footer.
        if (packet.size() < 1 + XCP_SYS_SIZE)
            return;
        parseSys(d);
        storeRaw(XCP_SYS_OFFSET, d, XCP_SYS_SIZE);
    } else if (pid == 4) {
        // ODT4: magnetometer (MMC5983MA, fw >= v1.15.0). Raw only -- every
        // field is reached through the A2L, so no hand-written parse is
        // needed. Missing this branch is what made the mag read a constant 0
        // even after the ODT was allocated: the frame arrived and was dropped.
        if (packet.size() < 1 + XCP_MAG_SIZE)
            return;
        storeRaw(XCP_MAG_OFFSET, d, XCP_MAG_SIZE);
    }
}

void XcpClient::handleResponse(const QByteArray &packet)
{
    if (!m_busy)
        return;                                  // unexpected datagram

    m_timeout->stop();

    const quint8 pid = quint8(packet[0]);
    if (pid == PID_ERR) {
        const quint8 code = quint8(packet.size() > 1 ? packet[1] : 0);
        emit errorOccurred(QString("XCP error code 0x%1%2")
                               .arg(code, 2, 16, QChar('0'))
                               .arg(code == 0x25 ? " (write protected)" : ""));
        if (m_current.type == ReqType::Connect || m_current.type == ReqType::GetId
            || m_current.type == ReqType::UploadId) {
            dropConnection("Connection setup failed");
            return;
        }
        if (m_current.type == ReqType::DaqCmd || m_current.type == ReqType::DaqStart) {
            // abort the remaining setup steps, panel falls back to polling
            QQueue<Request> keep;
            while (!m_queue.isEmpty()) {
                Request r = m_queue.dequeue();
                if (r.type != ReqType::DaqCmd && r.type != ReqType::DaqStart)
                    keep.enqueue(r);
            }
            m_queue = keep;
            emit daqFailed();
        }
        sendNext();
        return;
    }
    if (pid != PID_RES)
        return;                                  // DAQ etc. not used here

    switch (m_current.type) {
    case ReqType::Connect: {
        QByteArray cmd(2, '\0');
        cmd[0] = char(CMD_GET_ID);
        enqueue({ReqType::GetId, cmd, 0});
        break;
    }

    case ReqType::GetId:
        m_identLen = (packet.size() >= 8)
                         ? qFromLittleEndian<quint32>(packet.constData() + 4) : 0;
        if (m_identLen > 0 && m_identLen <= 63) {
            QByteArray up(2, '\0');
            up[0] = char(CMD_UPLOAD);
            up[1] = char(quint8(m_identLen));
            enqueue({ReqType::UploadId, up, 0});
        } else {
            m_connected = true;
            emit connected(QString());
        }
        break;

    case ReqType::UploadId:
        m_connected = true;
        emit connected(QString::fromLatin1(packet.mid(1, int(m_identLen))));
        break;

    case ReqType::Poll:
        // Core chunk: fill m_accum but do not emit yet — the chained PollImu
        // response emits the merged block (see pollMeasurements()).
        if (packet.size() >= 1 + XCP_DATA_SIZE) {
            const char *d = packet.constData() + 1;
            m_accum.valid    = (qFromLittleEndian<quint32>(d) == XCP_DATA_MAGIC);
            m_accum.verMajor = quint8(d[4]);
            m_accum.verMinor = quint8(d[5]);
            m_accum.verStep  = quint8(d[6]);
            m_accum.tickMs   = qFromLittleEndian<quint32>(d + 8);
            auto readF = [d](int off) {
                float f = 0.0f;
                std::memcpy(&f, d + off, sizeof(f));         // IEEE754 LE
                return f;
            };
            m_accum.dieTempC    = readF(12);
            m_accum.dtscTempC   = readF(16);
            m_accum.vddCore     = readF(20);
            m_accum.vddp3       = readF(24);
            m_accum.vext        = readF(28);
            m_accum.baroPresent = (quint8(d[35]) != 0);
            m_accum.diagStatus  = qFromLittleEndian<quint32>(d + 36);
            m_accum.baroPressPa = readF(40);
            m_accum.baroTempC   = readF(44);
            m_accum.baroAltM    = readF(48);
            m_verMajor = m_accum.verMajor;       // cache for DAQ frames
            m_verMinor = m_accum.verMinor;
            m_verStep  = m_accum.verStep;
            m_verValid = m_accum.valid;
            storeRaw(0, d, XCP_DATA_SIZE);
        }
        break;

    case ReqType::PollImu:
        // IMU chunk: fill m_accum only; the last chunk in the chain emits.
        if (packet.size() >= 1 + XCP_IMU_SIZE) {
            parseImu(packet.constData() + 1);
            storeRaw(XCP_IMU_OFFSET, packet.constData() + 1, XCP_IMU_SIZE);
        }
        break;

    case ReqType::PollAhrs:
        if (packet.size() >= 1 + XCP_AHRS_SIZE) {
            parseAhrs(packet.constData() + 1);
            storeRaw(XCP_AHRS_OFFSET, packet.constData() + 1, XCP_AHRS_SIZE);
        }
        break;

    case ReqType::PollSys:
        // Final chunk of the chain: complete m_accum and emit the merged block.
        if (packet.size() >= 1 + XCP_SYS_SIZE) {
            parseSys(packet.constData() + 1);
            storeRaw(XCP_SYS_OFFSET, packet.constData() + 1, XCP_SYS_SIZE);
        }
        emit measurementsReceived(m_accum);
        break;

    case ReqType::MemRead:
        emit memoryRead(m_current.addr, packet.mid(1));
        break;

    case ReqType::MemWrite:
        emit memoryWritten(m_current.addr);
        break;

    case ReqType::DaqCmd:
        break;                                   // intermediate setup step

    case ReqType::DaqStart:
        m_daqActive = true;
        emit daqStarted();
        break;
    }

    sendNext();
}

void XcpClient::onTimeout()
{
    dropConnection(m_connected ? QString("Timeout - connection lost")
                               : QString("No response from the board (connect timeout)"));
}

void XcpClient::dropConnection(const QString &reason)
{
    m_timeout->stop();
    m_queue.clear();
    m_busy      = false;
    m_connected = false;
    m_daqActive = false;
    m_verValid  = false;
    emit errorOccurred(reason);
    emit disconnected();
}
