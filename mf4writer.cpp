#include "mf4writer.h"

#include <QByteArray>
#include <QDataStream>
#include <QDateTime>
#include <QFile>

// MDF 4.10 block layout notes:
// - every block: char id[4] ("##XX"), u32 reserved, u64 length, u64 link_count,
//   then link_count u64 links, then block data. All blocks 8-byte aligned.
// - this file: ID, HD, FH, DG, CG, 8x CN, TX (names/units), DT (records).

namespace {

constexpr int RECORD_SIZE = 36;

struct ChannelDef {
    const char *name;
    const char *unit;       // nullptr = no unit
    quint8      cnType;     // 0 = fixed, 2 = master
    quint8      syncType;   // 1 = time (master), 0 = none
    quint8      dataType;   // 0 = uint LE, 4 = float LE
    quint32     byteOffset;
    quint32     bitCount;
};

constexpr ChannelDef CHANNELS[] = {
    {"time",          "s",    2, 1, 4, 0,  64},
    {"DieTemp_DTS",   "\xC2\xB0""C", 0, 0, 4, 8,  32},   // UTF-8 degree sign
    {"DieTemp_DTSC",  "\xC2\xB0""C", 0, 0, 4, 12, 32},
    {"VDD",           "V",    0, 0, 4, 16, 32},
    {"VDDP3",         "V",    0, 0, 4, 20, 32},
    {"VEXT",          "V",    0, 0, 4, 24, 32},
    {"TickMs",        "ms",   0, 0, 0, 28, 32},
    {"DiagStatus",    nullptr,0, 0, 0, 32, 32},
};
constexpr int NUM_CH = int(sizeof(CHANNELS) / sizeof(CHANNELS[0]));

qint64 alignedTxSize(const char *text)
{
    if (!text)
        return 0;
    const qint64 raw = 24 + qint64(strlen(text)) + 1;   // zero-terminated
    return (raw + 7) & ~qint64(7);
}

void putBlockHeader(QDataStream &s, const char id[5], qint64 length, qint64 linkCount)
{
    s.writeRawData(id, 4);
    s << quint32(0) << quint64(length) << quint64(linkCount);
}

qint64 putTx(QDataStream &s, qint64 offset, const char *text)
{
    const qint64 size = alignedTxSize(text);
    putBlockHeader(s, "##TX", 24 + qint64(strlen(text)) + 1, 0);
    s.writeRawData(text, int(strlen(text)) + 1);
    for (qint64 pad = 24 + qint64(strlen(text)) + 1; pad < size; ++pad)
        s << quint8(0);
    return offset + size;
}

} // namespace

bool Mf4Writer::save(const QString &path, QString *errorOut) const
{
    // ---- compute the fixed layout ----
    constexpr qint64 ID_SZ = 64,  HD_SZ = 104, FH_SZ = 56;
    constexpr qint64 DG_SZ = 64,  CG_SZ = 104, CN_SZ = 160;

    const qint64 offHd = ID_SZ;
    const qint64 offFh = offHd + HD_SZ;
    const qint64 offDg = offFh + FH_SZ;
    const qint64 offCg = offDg + DG_SZ;
    const qint64 offCn0 = offCg + CG_SZ;

    qint64 offCn[NUM_CH], offTxName[NUM_CH], offTxUnit[NUM_CH];
    for (int i = 0; i < NUM_CH; ++i)
        offCn[i] = offCn0 + i * CN_SZ;

    qint64 cursor = offCn0 + NUM_CH * CN_SZ;
    for (int i = 0; i < NUM_CH; ++i) {
        offTxName[i] = cursor;
        cursor += alignedTxSize(CHANNELS[i].name);
        offTxUnit[i] = CHANNELS[i].unit ? cursor : 0;
        cursor += alignedTxSize(CHANNELS[i].unit);
    }
    const qint64 offDt = cursor;
    const qint64 dtSize = 24 + qint64(m_samples.size()) * RECORD_SIZE;

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
      << quint64(m_samples.size())              // cycle count
      << quint16(0) << quint16(0) << quint32(0)
      << quint32(RECORD_SIZE) << quint32(0);

    // CN blocks
    for (int i = 0; i < NUM_CH; ++i) {
        const ChannelDef &ch = CHANNELS[i];
        putBlockHeader(s, "##CN", CN_SZ, 8);
        s << quint64(i + 1 < NUM_CH ? offCn[i + 1] : 0)   // cn_next
          << quint64(0)                                   // composition
          << quint64(offTxName[i])                        // tx_name
          << quint64(0)                                   // si_source
          << quint64(0)                                   // cc_conversion (1:1)
          << quint64(0)                                   // data
          << quint64(offTxUnit[i])                        // md_unit
          << quint64(0);                                  // md_comment
        s << ch.cnType << ch.syncType << ch.dataType << quint8(0)
          << ch.byteOffset << ch.bitCount
          << quint32(0)                                   // flags
          << quint32(0)                                   // invalid bit pos
          << quint8(0) << quint8(0) << quint16(0);
        s.setFloatingPointPrecision(QDataStream::DoublePrecision);
        for (int k = 0; k < 6; ++k) s << double(0.0);     // ranges/limits
        s.setFloatingPointPrecision(QDataStream::SinglePrecision);
    }

    // TX blocks (names + units)
    qint64 txCursor = offCn0 + NUM_CH * CN_SZ;
    for (int i = 0; i < NUM_CH; ++i) {
        txCursor = putTx(s, txCursor, CHANNELS[i].name);
        if (CHANNELS[i].unit)
            txCursor = putTx(s, txCursor, CHANNELS[i].unit);
    }

    // DT block with the records
    putBlockHeader(s, "##DT", dtSize, 0);
    for (const Sample &sm : m_samples) {
        s.setFloatingPointPrecision(QDataStream::DoublePrecision);
        s << sm.t;
        s.setFloatingPointPrecision(QDataStream::SinglePrecision);
        s << sm.dts << sm.dtsc << sm.vdd << sm.vddp3 << sm.vext
          << sm.tick << sm.diag;
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
