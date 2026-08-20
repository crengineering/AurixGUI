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

// Largest payload one packet can carry: MAX_CTO / MAX_DTO is 64, minus the PID
// byte. Applies to both a SHORT_UPLOAD response and a DAQ ODT.
constexpr int     XCP_CHUNK_MAX  = 63;

// Offsets of the sub-structures inside Xcp_Data, used ONLY by the named-field
// convenience layer (parseNamedFields) that feeds the footer and the older
// tabs. The transport no longer knows about them: it moves the whole block in
// XCP_CHUNK_MAX-sized pieces whose count comes from the A2L. Anything described
// in the A2L is read straight out of Measurements::raw and needs nothing here.
constexpr int     XCP_IMU_OFFSET  = 52;         // IMU sub-block at 0x34
constexpr int     XCP_AHRS_OFFSET = 84;         // attitude sub-block at 0x54
constexpr int     XCP_SYS_OFFSET  = 124;        // core + Ethernet stats at 0x7C
constexpr int     XCP_MAG_OFFSET  = 180;        // magnetometer sub-block at 0xB4
constexpr int     XCP_GNSS_OFFSET = 204;        // GNSS sub-block at 0xCC

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

QVector<QPair<int, int>> XcpClient::blockChunks() const
{
    QVector<QPair<int, int>> chunks;
    for (int off = 0; off < m_blockSize; off += XCP_CHUNK_MAX)
        chunks.append({off, qMin(XCP_CHUNK_MAX, m_blockSize - off)});
    return chunks;
}

void XcpClient::pollMeasurements(quint32 address, int blockSize)
{
    if (!m_connected || blockSize <= 0)
        return;

    // never let polls pile up behind a slow link
    for (const Request &r : m_queue)
        if (r.type == ReqType::PollChunk)
            return;
    if (m_busy && m_current.type == ReqType::PollChunk)
        return;

    m_blockBase = address;
    m_blockSize = blockSize;

    // The block exceeds MAX_CTO(64), so read it as consecutive SHORT_UPLOADs.
    // The chunk count follows blockSize, which comes from the A2L -- nothing
    // here names a peripheral, so a new measurement is covered automatically.
    const QVector<QPair<int, int>> chunks = blockChunks();
    for (int i = 0; i < chunks.size(); ++i) {
        const int off = chunks[i].first;
        const int len = chunks[i].second;

        QByteArray cmd(8, '\0');
        cmd[0] = char(CMD_SHORT_UPLOAD);
        cmd[1] = char(len);
        qToLittleEndian<quint32>(address + quint32(off), cmd.data() + 4);

        Request r{ReqType::PollChunk, cmd, address + quint32(off), off, len,
                  (i == chunks.size() - 1)};
        enqueue(r);
    }
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

void XcpClient::startDaq(quint32 baseAddress, int blockSize)
{
    if (!m_connected || blockSize <= 0)
        return;

    m_blockBase = baseAddress;
    m_blockSize = blockSize;

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

    // The whole block, split into consecutive ODTs of at most XCP_CHUNK_MAX
    // bytes. Both the count and the extents follow blockSize, which the caller
    // took from the A2L -- so every byte any MEASUREMENT describes is in the
    // stream by construction.
    //
    // That matters because after startDaq() the client stops polling: anything
    // not in an ODT never updates again. Two bugs came from hand-maintaining
    // this list (the core-load footer read 0 % until ODT2/ODT3 were added, and
    // the magnetometer read a constant 0 because its frames had nowhere to go).
    // Deriving it removes that whole class.
    const QVector<QPair<int, int>> chunks = blockChunks();
    m_lastOdt = chunks.size() - 1;

    simple({CMD_FREE_DAQ});
    simple({CMD_ALLOC_DAQ, 0x00, 0x01, 0x00});                   // 1 list
    simple({CMD_ALLOC_ODT, 0x00, 0x00, 0x00, quint8(chunks.size())});
    for (int i = 0; i < chunks.size(); ++i)
        simple({CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, quint8(i), 0x01});

    for (int i = 0; i < chunks.size(); ++i) {
        simple({CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, quint8(i), 0x00});
        writeDaq(quint8(chunks[i].second),
                 baseAddress + quint32(chunks[i].first));
    }

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
    // Buffer sizes itself to the block the A2L asked for, so it always covers
    // every described measurement.
    if (m_accum.raw.size() != m_blockSize)
        m_accum.raw = QByteArray(m_blockSize, char(0));
    if (offset >= 0 && len > 0 && offset + len <= m_blockSize)
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

void XcpClient::parseNamedFields()
{
    // Convenience layer only: fills the C++ struct members the footer and the
    // older tabs read. Everything described in the A2L is decoded from
    // Measurements::raw instead, so nothing needs adding here for a new signal.
    if (m_accum.raw.size() < m_blockSize)
        return;

    const char *d = m_accum.raw.constData();

    m_accum.valid    = (qFromLittleEndian<quint32>(d) == XCP_DATA_MAGIC);
    m_accum.verMajor = quint8(d[4]);
    m_accum.verMinor = quint8(d[5]);
    m_accum.verStep  = quint8(d[6]);
    m_accum.tickMs   = qFromLittleEndian<quint32>(d + 8);

    auto readF = [d](int off) {
        float f = 0.0f;
        std::memcpy(&f, d + off, sizeof(f));            // IEEE754 LE
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

    if (m_blockSize >= XCP_IMU_OFFSET + 32)
        parseImu(d + XCP_IMU_OFFSET);
    if (m_blockSize >= XCP_AHRS_OFFSET + 40)
        parseAhrs(d + XCP_AHRS_OFFSET);
    if (m_blockSize >= XCP_SYS_OFFSET + 56)
        parseSys(d + XCP_SYS_OFFSET);

    // Magnetometer and GNSS: only the presence byte gets a named field, for
    // the footer's lamps. Guarded on the block size like the sub-blocks above,
    // so firmware built before these existed reports "not present" rather than
    // reading past the end of the block.
    m_accum.magPresent  = (m_blockSize > XCP_MAG_OFFSET)
                          && quint8(d[XCP_MAG_OFFSET]) != 0;
    m_accum.gnssPresent = (m_blockSize > XCP_GNSS_OFFSET)
                          && quint8(d[XCP_GNSS_OFFSET]) != 0;

    m_verMajor = m_accum.verMajor;      // cached for DAQ frames, which carry
    m_verMinor = m_accum.verMinor;      // no version of their own
    m_verStep  = m_accum.verStep;
    m_verValid = m_accum.valid;
}

void XcpClient::handleDaqFrame(const QByteArray &packet)
{
    if (packet.isEmpty())
        return;

    const quint8 pid = quint8(packet[0]);
    const char  *d   = packet.constData() + 1;

    // The PID is the ODT index, and startDaq() laid the ODTs out as consecutive
    // slices of the block, so the PID alone gives the destination offset. No
    // per-peripheral branch, and therefore no frame that can arrive with
    // nowhere to be stored.
    const QVector<QPair<int, int>> chunks = blockChunks();
    if (pid >= chunks.size())
        return;                             // stale DAQ list from a previous run

    const int off = chunks[pid].first;
    const int len = chunks[pid].second;
    if (packet.size() < 1 + len)
        return;

    storeRaw(off, d, len);

    // Emit once per event, after the final ODT of the cycle. The firmware sends
    // them in index order, so by here the block is whole -- which also removes
    // the one-frame lag the old ODT0-emits scheme had on every sub-block.
    if (int(pid) == m_lastOdt) {
        // ODT0 now starts at offset 0, so the magic word and version travel in
        // the DAQ stream too -- no need to cache them from a prior poll.
        parseNamedFields();
        emit measurementsReceived(m_accum);
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

    case ReqType::PollChunk:
        // One slice of the block. Each chunk only stores; the last one in the
        // sequence decodes the named fields and emits the merged block, so the
        // consumer always sees a consistent snapshot.
        if (packet.size() >= 1 + m_current.len)
            storeRaw(m_current.off, packet.constData() + 1, m_current.len);

        if (m_current.last) {
            parseNamedFields();
            emit measurementsReceived(m_accum);
        }
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
