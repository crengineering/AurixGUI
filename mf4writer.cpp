#include "mf4writer.h"

#include <QByteArray>
#include <QDataStream>
#include <QDateTime>
#include <QFile>

// MDF 4.10 block layout notes:
// - every block: char id[4] ("##XX"), u32 reserved, u64 length, u64 link_count,
//   then link_count u64 links, then block data. All blocks 8-byte aligned.
// - this file: ID, HD, FH, DG, CG, N x CN, TX (names/units), DT (records).
// - record layout: time (double, 8 B) then each channel (float or uint32, 4 B).

namespace {

// One entry in the file's channel list (time master + selected data channels).
struct Desc {
    QByteArray name;
    QByteArray unit;        // empty = no unit block
    quint8     cnType;      // 0 = fixed, 2 = master
    quint8     syncType;    // 1 = time (master), 0 = none
    quint8     dataType;    // 0 = uint LE, 4 = float LE
    quint32    byteOffset;
    quint32    bitCount;
};

qint64 alignedTxSize(const QByteArray &text)
{
    if (text.isEmpty())
        return 0;
    const qint64 raw = 24 + qint64(text.size()) + 1;   // zero-terminated
    return (raw + 7) & ~qint64(7);
}

void putBlockHeader(QDataStream &s, const char id[5], qint64 length, qint64 linkCount)
{
    s.writeRawData(id, 4);
    s << quint32(0) << quint64(length) << quint64(linkCount);
}

qint64 putTx(QDataStream &s, qint64 offset, const QByteArray &text)
{
    const qint64 size = alignedTxSize(text);
    putBlockHeader(s, "##TX", 24 + qint64(text.size()) + 1, 0);
    s.writeRawData(text.constData(), text.size() + 1);   // include terminating NUL
    for (qint64 pad = 24 + qint64(text.size()) + 1; pad < size; ++pad)
        s << quint8(0);
    return offset + size;
}

} // namespace

void Mf4Writer::begin(const QVector<Channel> &channels)
{
    m_channels = channels;
    m_time.clear();
    m_rows.clear();
}

void Mf4Writer::clear()
{
    m_time.clear();
    m_rows.clear();
}

void Mf4Writer::append(double t, const QVector<double> &values)
{
    if (values.size() != m_channels.size())
        return;                     // guard against a mismatched row
    m_time.append(t);
    m_rows.append(values);
}

bool Mf4Writer::save(const QString &path, QString *errorOut) const
{
    const int    N          = m_channels.size();
    const int    numCh      = N + 1;                 // + time master
    const qint64 recordSize = 8 + qint64(N) * 4;

    // ---- build the channel descriptor list (master first) ----
    QVector<Desc> ch;
    ch.append({ QByteArray("time"), QByteArray("s"), 2, 1, 4, 0, 64 });
    for (int i = 0; i < N; ++i) {
        ch.append({ m_channels[i].name.toUtf8(),
                    m_channels[i].unit.toUtf8(),
                    0, 0, quint8(m_channels[i].isFloat ? 4 : 0),
                    quint32(8 + i * 4), 32 });
    }

    // ---- compute the layout ----
    constexpr qint64 ID_SZ = 64,  HD_SZ = 104, FH_SZ = 56;
    constexpr qint64 DG_SZ = 64,  CG_SZ = 104, CN_SZ = 160;

    const qint64 offHd  = ID_SZ;
    const qint64 offFh  = offHd + HD_SZ;
    const qint64 offDg  = offFh + FH_SZ;
    const qint64 offCg  = offDg + DG_SZ;
    const qint64 offCn0 = offCg + CG_SZ;

    QVector<qint64> offCn(numCh), offTxName(numCh), offTxUnit(numCh);
    for (int i = 0; i < numCh; ++i)
        offCn[i] = offCn0 + qint64(i) * CN_SZ;

    qint64 cursor = offCn0 + qint64(numCh) * CN_SZ;
    for (int i = 0; i < numCh; ++i) {
        offTxName[i] = cursor;
        cursor += alignedTxSize(ch[i].name);
        offTxUnit[i] = ch[i].unit.isEmpty() ? 0 : cursor;
        cursor += alignedTxSize(ch[i].unit);
    }
    const qint64 offDt  = cursor;
    const qint64 dtSize = 24 + qint64(m_time.size()) * recordSize;

    // ---- emit ----
    QByteArray buf;
    buf.reserve(int(offDt + dtSize));
    QDataStream s(&buf, QIODevice::WriteOnly);
    s.setByteOrder(QDataStream::LittleEndian);
    s.setFloatingPointPrecision(QDataStream::SinglePrecision);

    // ID block
    s.writeRawData("MDF     ", 8);
    s.writeRawData("4.10    ", 8);
    s.writeRawData("AurixGUI", 8);
    s << quint32(0);
    s << quint16(410);
    for (int i = 0; i < 34; ++i) s << quint8(0);   // pad ID block to 64 bytes

    // HD block
    const quint64 nowNs = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000000ull;
    putBlockHeader(s, "##HD", HD_SZ, 6);
    s << quint64(offDg) << quint64(offFh) << quint64(0) << quint64(0)
      << quint64(0) << quint64(0);
    s << nowNs << qint16(0) << qint16(0) << quint8(0) << quint8(0)
      << quint8(0) << quint8(0);
    s.setFloatingPointPrecision(QDataStream::DoublePrecision);
    s << double(0.0) << double(0.0);            // start angle / distance
    s.setFloatingPointPrecision(QDataStream::SinglePrecision);

    // FH block
    putBlockHeader(s, "##FH", FH_SZ, 2);
    s << quint64(0) << quint64(0);
    s << nowNs << qint16(0) << qint16(0) << quint8(0)
      << quint8(0) << quint8(0) << quint8(0);

    // DG block
    putBlockHeader(s, "##DG", DG_SZ, 4);
    s << quint64(0) << quint64(offCg) << quint64(offDt) << quint64(0);
    s << quint8(0);                             // record id size
    for (int i = 0; i < 7; ++i) s << quint8(0);

    // CG block
    putBlockHeader(s, "##CG", CG_SZ, 6);
    s << quint64(0) << quint64(offCn[0]) << quint64(0) << quint64(0)
      << quint64(0) << quint64(0);
    s << quint64(0)                             // record id
      << quint64(m_time.size())                 // cycle count
      << quint16(0) << quint16(0) << quint32(0)
      << quint32(recordSize) << quint32(0);

    // CN blocks
    for (int i = 0; i < numCh; ++i) {
        putBlockHeader(s, "##CN", CN_SZ, 8);
        s << quint64(i + 1 < numCh ? offCn[i + 1] : 0)  // cn_next
          << quint64(0)                                 // composition
          << quint64(offTxName[i])                      // tx_name
          << quint64(0)                                 // si_source
          << quint64(0)                                 // cc_conversion (1:1)
          << quint64(0)                                 // data
          << quint64(offTxUnit[i])                      // md_unit
          << quint64(0);                                // md_comment
        s << ch[i].cnType << ch[i].syncType << ch[i].dataType << quint8(0)
          << ch[i].byteOffset << ch[i].bitCount
          << quint32(0)                                 // flags
          << quint32(0)                                 // invalid bit pos
          << quint8(0) << quint8(0) << quint16(0);
        s.setFloatingPointPrecision(QDataStream::DoublePrecision);
        for (int k = 0; k < 6; ++k) s << double(0.0);   // ranges/limits
        s.setFloatingPointPrecision(QDataStream::SinglePrecision);
    }

    // TX blocks (names + units)
    qint64 txCursor = offCn0 + qint64(numCh) * CN_SZ;
    for (int i = 0; i < numCh; ++i) {
        txCursor = putTx(s, txCursor, ch[i].name);
        if (!ch[i].unit.isEmpty())
            txCursor = putTx(s, txCursor, ch[i].unit);
    }

    // DT block with the records
    putBlockHeader(s, "##DT", dtSize, 0);
    for (int r = 0; r < m_time.size(); ++r) {
        s.setFloatingPointPrecision(QDataStream::DoublePrecision);
        s << m_time[r];
        s.setFloatingPointPrecision(QDataStream::SinglePrecision);
        const QVector<double> &row = m_rows[r];
        for (int i = 0; i < N; ++i) {
            if (m_channels[i].isFloat)
                s << float(row[i]);
            else
                s << quint32(row[i]);
        }
    }

    // ---- write to disk ----
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        if (errorOut)
            *errorOut = f.errorString();
        return false;
    }
    f.write(buf);
    f.close();
    return true;
}
