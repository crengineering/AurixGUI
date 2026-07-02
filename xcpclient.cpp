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
constexpr int     XCP_DATA_SIZE  = 40;          // v1.2.0 block
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

    QByteArray cmd(8, '\0');
    cmd[0] = char(CMD_SHORT_UPLOAD);
    cmd[1] = char(XCP_DATA_SIZE);
    qToLittleEndian<quint32>(address, cmd.data() + 4);
    enqueue({ReqType::Poll, cmd, address});
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

void XcpClient::startDaq(quint32 entryAddress, quint8 entrySize)
{
    if (!m_connected)
        return;

    auto simple = [this](std::initializer_list<quint8> bytes) {
        QByteArray p;
        for (quint8 b : bytes)
            p.append(char(b));
        enqueue({ReqType::DaqCmd, p, 0});
    };

    simple({CMD_FREE_DAQ});
    simple({CMD_ALLOC_DAQ, 0x00, 0x01, 0x00});                  // 1 list
    simple({CMD_ALLOC_ODT, 0x00, 0x00, 0x00, 0x01});            // 1 ODT
    simple({CMD_ALLOC_ODT_ENTRY, 0x00, 0x00, 0x00, 0x00, 0x01});// 1 entry
    simple({CMD_SET_DAQ_PTR, 0x00, 0x00, 0x00, 0x00, 0x00});

    QByteArray wr(8, '\0');
    wr[0] = char(CMD_WRITE_DAQ);
    wr[1] = char(0xFF);                                          // no bit offset
    wr[2] = char(entrySize);
    qToLittleEndian<quint32>(entryAddress, wr.data() + 4);
    enqueue({ReqType::DaqCmd, wr, 0});

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

void XcpClient::handleDaqFrame(const QByteArray &packet)
{
    // single ODT 0: tick u32, dts f, dtsc f, vdd f, vddp3 f, vext f,
    // raw codes u32, diagStatus u32  (= Xcp_Data + 8, 32 bytes)
    if (quint8(packet[0]) != 0 || packet.size() < 1 + 32)
        return;

    const char *d = packet.constData() + 1;
    Measurements m;
    m.valid    = m_verValid;
    m.verMajor = m_verMajor;
    m.verMinor = m_verMinor;
    m.verStep  = m_verStep;
    m.tickMs   = qFromLittleEndian<quint32>(d);
    auto readF = [d](int off) {
        float f = 0.0f;
        std::memcpy(&f, d + off, sizeof(f));
        return f;
    };
    m.dieTempC   = readF(4);
    m.dtscTempC  = readF(8);
    m.vddCore    = readF(12);
    m.vddp3      = readF(16);
    m.vext       = readF(20);
    m.diagStatus = qFromLittleEndian<quint32>(d + 28);
    emit measurementsReceived(m);
}

void XcpClient::handleResponse(const QByteArray &packet)
{
    if (!m_busy)
        return;                                  // unexpected datagram

    m_timeout->stop();

    const quint8 pid = quint8(packet[0]);
    if (pid == PID_ERR) {
        const quint8 code = quint8(packet.size() > 1 ? packet[1] : 0);
        emit errorOccurred(QString("XCP-Fehlercode 0x%1%2")
                               .arg(code, 2, 16, QChar('0'))
                               .arg(code == 0x25 ? " (schreibgeschuetzt)" : ""));
        if (m_current.type == ReqType::Connect || m_current.type == ReqType::GetId
            || m_current.type == ReqType::UploadId) {
            dropConnection("Verbindungsaufbau fehlgeschlagen");
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
        if (packet.size() >= 1 + XCP_DATA_SIZE) {
            const char *d = packet.constData() + 1;
            Measurements m;
            m.valid    = (qFromLittleEndian<quint32>(d) == XCP_DATA_MAGIC);
            m.verMajor = quint8(d[4]);
            m.verMinor = quint8(d[5]);
            m.verStep  = quint8(d[6]);
            m.tickMs   = qFromLittleEndian<quint32>(d + 8);
            auto readF = [d](int off) {
                float f = 0.0f;
                std::memcpy(&f, d + off, sizeof(f));         // IEEE754 LE
                return f;
            };
            m.dieTempC   = readF(12);
            m.dtscTempC  = readF(16);
            m.vddCore    = readF(20);
            m.vddp3      = readF(24);
            m.vext       = readF(28);
            m.diagStatus = qFromLittleEndian<quint32>(d + 36);
            m_verMajor = m.verMajor;             // cache for DAQ frames
            m_verMinor = m.verMinor;
            m_verStep  = m.verStep;
            m_verValid = m.valid;
            emit measurementsReceived(m);
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
    dropConnection(m_connected ? QString("Timeout - Verbindung verloren")
                               : QString("Keine Antwort vom Board (Timeout beim Verbinden)"));
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
