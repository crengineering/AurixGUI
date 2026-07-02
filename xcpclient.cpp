#include "xcpclient.h"

#include <QTimer>
#include <QtEndian>
#include <cstring>

namespace {
constexpr quint8 CMD_CONNECT      = 0xFF;
constexpr quint8 CMD_DISCONNECT   = 0xFE;
constexpr quint8 CMD_GET_ID       = 0xFA;
constexpr quint8 CMD_UPLOAD       = 0xF5;
constexpr quint8 CMD_SHORT_UPLOAD = 0xF4;

constexpr quint8 PID_RES = 0xFF;
constexpr quint8 PID_ERR = 0xFE;

constexpr quint32 XCP_DATA_MAGIC = 0x41555258;  // must match Measurements.h
constexpr int     XCP_DATA_SIZE  = 16;
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
    if (m_state != State::Idle)
        disconnectFromSlave();

    m_host = QHostAddress(host);
    m_port = port;
    m_ctr  = 0;

    if (m_host.isNull()) {
        emit errorOccurred(QString("Ungueltige IP-Adresse: %1").arg(host));
        return;
    }

    m_state = State::Connecting;
    QByteArray cmd(2, '\0');
    cmd[0] = char(CMD_CONNECT);
    cmd[1] = 0;                                  // mode: normal
    sendCommand(cmd);
}

void XcpClient::disconnectFromSlave()
{
    if (m_state == State::Connected)
        sendCommand(QByteArray(1, char(CMD_DISCONNECT)));  // fire and forget

    m_timeout->stop();
    m_state = State::Idle;
    m_pollPending = false;
    emit disconnected();
}

void XcpClient::pollMeasurements(quint32 address)
{
    if (m_state != State::Connected || m_pollPending)
        return;                                  // previous request still open

    QByteArray cmd(8, '\0');
    cmd[0] = char(CMD_SHORT_UPLOAD);
    cmd[1] = char(XCP_DATA_SIZE);                // number of bytes
    cmd[3] = 0;                                  // address extension
    qToLittleEndian<quint32>(address, cmd.data() + 4);

    m_pollPending = true;
    sendCommand(cmd);
}

void XcpClient::sendCommand(const QByteArray &packet)
{
    QByteArray frame(4, '\0');
    qToLittleEndian<quint16>(quint16(packet.size()), frame.data());
    qToLittleEndian<quint16>(m_ctr++, frame.data() + 2);
    frame += packet;

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

        handleResponse(dgram.mid(4, len));
    }
}

void XcpClient::handleResponse(const QByteArray &packet)
{
    m_timeout->stop();

    const quint8 pid = quint8(packet[0]);
    if (pid == PID_ERR) {
        emit errorOccurred(QString("XCP-Fehlercode 0x%1")
                               .arg(quint8(packet.size() > 1 ? packet[1] : 0), 2, 16, QChar('0')));
        m_pollPending = false;
        return;
    }
    if (pid != PID_RES)
        return;                                  // DAQ etc. not used here

    switch (m_state) {
    case State::Connecting: {
        // CONNECT response received -> fetch the ident string next
        m_state = State::GettingId;
        QByteArray cmd(2, '\0');
        cmd[0] = char(CMD_GET_ID);
        cmd[1] = 0;                              // mode 0
        sendCommand(cmd);
        break;
    }

    case State::GettingId:
        if (packet.size() >= 8) {
            m_identLen = qFromLittleEndian<quint32>(packet.constData() + 4);
            if (m_identLen > 0 && m_identLen <= 63) {
                m_state = State::UploadingId;
                QByteArray up(2, '\0');
                up[0] = char(CMD_UPLOAD);
                up[1] = char(quint8(m_identLen));
                sendCommand(up);
            } else {
                m_state = State::Connected;
                emit connected(QString());
            }
        }
        break;

    case State::UploadingId:
        m_state = State::Connected;
        emit connected(QString::fromLatin1(packet.mid(1, int(m_identLen))));
        break;

    case State::Connected:
        if (m_pollPending && packet.size() >= 1 + XCP_DATA_SIZE) {
            const char *d = packet.constData() + 1;
            Measurements m;
            const quint32 magic = qFromLittleEndian<quint32>(d);
            m.valid    = (magic == XCP_DATA_MAGIC);
            m.verMajor = quint8(d[4]);
            m.verMinor = quint8(d[5]);
            m.verStep  = quint8(d[6]);
            m.tickMs   = qFromLittleEndian<quint32>(d + 8);
            float temp = 0.0f;
            std::memcpy(&temp, d + 12, sizeof(temp));        // IEEE754 LE
            m.dieTempC = temp;
            emit measurementsReceived(m);
        }
        m_pollPending = false;
        break;

    case State::Idle:
        break;
    }
}

void XcpClient::onTimeout()
{
    m_pollPending = false;

    if (m_state != State::Idle) {
        const bool wasConnecting = (m_state != State::Connected);
        m_state = State::Idle;
        emit errorOccurred(wasConnecting
                               ? QString("Keine Antwort vom Board (Timeout beim Verbinden)")
                               : QString("Timeout - Verbindung verloren"));
        emit disconnected();
    }
}
