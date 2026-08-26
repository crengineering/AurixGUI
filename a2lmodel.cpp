#include "a2lmodel.h"

#include <QFile>
#include <QHash>
#include <QMap>
#include <QtEndian>
#include <cstring>

namespace {

// Remove /* ... */ block comments (A2L uses only these). Each comment is
// replaced by a space so it cannot glue two tokens together.
QString stripBlockComments(const QString &in)
{
    QString out;
    out.reserve(in.size());
    int i = 0;
    const int n = in.size();
    while (i < n) {
        if (i + 1 < n && in[i] == '/' && in[i + 1] == '*') {
            i += 2;
            while (i + 1 < n && !(in[i] == '*' && in[i + 1] == '/'))
                ++i;
            i += 2;                 // skip the closing */
            out += QLatin1Char(' ');
        } else {
            out += in[i];
            ++i;
        }
    }
    return out;
}

// Split into whitespace-separated tokens; a "quoted string" becomes one token
// with the quotes stripped, so descriptions with spaces stay intact and can
// never be confused with keywords like VALUE.
QVector<QString> tokenize(const QString &text)
{
    QVector<QString> toks;
    int i = 0;
    const int n = text.size();
    while (i < n) {
        const QChar c = text[i];
        if (c.isSpace()) {
            ++i;
        } else if (c == QLatin1Char('"')) {
            ++i;
            QString s;
            while (i < n && text[i] != QLatin1Char('"')) {
                s += text[i];
                ++i;
            }
            ++i;                    // skip closing quote
            toks.append(s);
        } else {
            QString s;
            while (i < n && !text[i].isSpace()) {
                s += text[i];
                ++i;
            }
            toks.append(s);
        }
    }
    return toks;
}

bool mapBaseType(const QString &bt, A2lType *out)
{
    if (bt == QLatin1String("FLOAT32_IEEE")) { *out = A2lType::Float32; return true; }
    if (bt == QLatin1String("ULONG"))        { *out = A2lType::Uint32;  return true; }
    if (bt == QLatin1String("UWORD"))        { *out = A2lType::Uint16;  return true; }
    if (bt == QLatin1String("UBYTE"))        { *out = A2lType::Uint8;   return true; }
    return false;                   // other base types are not needed here
}

} // namespace

QVector<A2lChar> A2lModel::parseFile(const QString &path, QString *err)
{
    QVector<A2lChar> out;

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (err) *err = f.errorString();
        return out;
    }

    const QString text = stripBlockComments(QString::fromUtf8(f.readAll()));
    const QVector<QString> t = tokenize(text);

    // Pass 1: RECORD_LAYOUT name -> base type (from its FNC_VALUES entry).
    QHash<QString, A2lType> layouts;
    for (int i = 0; i + 2 < t.size(); ++i) {
        if (t[i] == QLatin1String("/begin") &&
            t[i + 1] == QLatin1String("RECORD_LAYOUT")) {
            const QString name = t[i + 2];
            for (int j = i + 3; j < t.size(); ++j) {
                if (t[j] == QLatin1String("/end"))
                    break;
                if (t[j] == QLatin1String("FNC_VALUES") && j + 2 < t.size()) {
                    A2lType ty;
                    if (mapBaseType(t[j + 2], &ty))
                        layouts.insert(name, ty);
                    break;
                }
            }
        }
    }

    // Pass 2: each CHARACTERISTIC block -> one A2lChar (if we know its layout).
    for (int i = 0; i + 3 < t.size(); ++i) {
        if (t[i] != QLatin1String("/begin") ||
            t[i + 1] != QLatin1String("CHARACTERISTIC"))
            continue;

        A2lChar c;
        c.name = t[i + 2];
        c.desc = t[i + 3];
        QString layoutName;
        bool haveValue = false;

        int depth = 1;
        int j = i + 4;
        for (; j < t.size() && depth > 0; ++j) {
            const QString &tok = t[j];
            if (tok == QLatin1String("/begin")) { ++depth; continue; }
            if (tok == QLatin1String("/end"))   { --depth; continue; }
            if (depth != 1)
                continue;
            // VALUE <addr> <recordLayout> <maxDiff> <conv> <lo> <hi>
            if (tok == QLatin1String("VALUE") && !haveValue && j + 6 < t.size()) {
                c.addr     = t[j + 1].toUInt(nullptr, 0);
                layoutName = t[j + 2];
                c.lo       = t[j + 5].toDouble();
                c.hi       = t[j + 6].toDouble();
                haveValue  = true;
            } else if (tok == QLatin1String("PHYS_UNIT") && j + 1 < t.size()) {
                c.unit = t[j + 1];
            }
        }

        if (haveValue && layouts.contains(layoutName)) {
            c.type = layouts.value(layoutName);
            out.append(c);
        }
        i = j - 1;                  // resume after this block
    }

    if (out.isEmpty() && err && err->isEmpty())
        *err = QStringLiteral("no CHARACTERISTIC entries found");
    return out;
}

QVector<A2lMeas> A2lModel::parseMeasurements(const QString &path, QString *err)
{
    QVector<A2lMeas> out;

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (err) *err = f.errorString();
        return out;
    }

    const QString text = stripBlockComments(QString::fromUtf8(f.readAll()));
    const QVector<QString> t = tokenize(text);

    // MEASUREMENT <name> <desc> <baseType> <conv> <res> <accuracy> <lo> <hi>
    // then optional ECU_ADDRESS / PHYS_UNIT / BIT_MASK lines.
    for (int i = 0; i + 9 < t.size(); ++i) {
        if (t[i] != QLatin1String("/begin") ||
            t[i + 1] != QLatin1String("MEASUREMENT"))
            continue;

        A2lMeas m;
        m.name = t[i + 2];
        m.desc = t[i + 3];
        if (!mapBaseType(t[i + 4], &m.type))
            continue;                       // unsupported base type: skip
        m.lo = t[i + 8].toDouble();
        m.hi = t[i + 9].toDouble();

        bool haveAddr = false;
        int depth = 1;
        int j = i + 10;
        for (; j < t.size() && depth > 0; ++j) {
            const QString &tok = t[j];
            if (tok == QLatin1String("/begin")) { ++depth; continue; }
            if (tok == QLatin1String("/end"))   { --depth; continue; }
            if (depth != 1)
                continue;
            if (tok == QLatin1String("ECU_ADDRESS") && j + 1 < t.size()) {
                m.addr   = t[j + 1].toUInt(nullptr, 0);
                haveAddr = true;
            } else if (tok == QLatin1String("PHYS_UNIT") && j + 1 < t.size()) {
                m.unit = t[j + 1];
            } else if (tok == QLatin1String("BIT_MASK")) {
                m.isBitMask = true;
                if (j + 1 < t.size())
                    m.bitMask = t[j + 1].toUInt(nullptr, 0);
            }
        }

        if (haveAddr)
            out.append(m);
        i = j - 1;                          // resume after this block
    }

    if (out.isEmpty() && err && err->isEmpty())
        *err = QStringLiteral("no MEASUREMENT entries found");
    return out;
}

bool A2lModel::decode(const QByteArray &raw, quint32 blockBase,
                      const A2lMeas &m, double *out)
{
    const int off = int(m.addr - blockBase);
    const int len = a2lTypeSize(m.type);
    if (off < 0 || raw.size() < off + len)
        return false;

    const char *d = raw.constData() + off;
    switch (m.type) {
    case A2lType::Float32: { float f = 0.0f; std::memcpy(&f, d, sizeof(f)); *out = double(f); break; }
    case A2lType::Uint32:  *out = double(qFromLittleEndian<quint32>(d)); break;
    case A2lType::Uint16:  *out = double(qFromLittleEndian<quint16>(d)); break;
    case A2lType::Uint8:   *out = double(quint8(*d));                    break;
    }
    return true;
}

QVector<A2lModel::BlockRange> A2lModel::blockRanges(const QVector<A2lMeas> &meas)
{
    // Group by the 256-byte-aligned base, which is how the firmware pins them.
    // QMap rather than QHash: it iterates in key order, so the ranges come out
    // sorted by address and the DAQ ODTs are laid out in a predictable order.
    QMap<quint32, int> extents;

    for (const A2lMeas &m : meas) {
        const quint32 base = m.addr & ~quint32(0xFF);
        const int     end  = int(m.addr - base) + a2lTypeSize(m.type);

        if (end > extents.value(base, 0))
            extents[base] = end;
    }

    QVector<BlockRange> ranges;
    ranges.reserve(int(extents.size()));
    for (auto it = extents.constBegin(); it != extents.constEnd(); ++it)
        ranges.append(BlockRange{it.key(), it.value()});

    return ranges;
}

bool A2lModel::decodeFrom(const QVector<quint32> &bases,
                          const QVector<QByteArray> &raws,
                          const A2lMeas &m, double *out)
{
    for (int i = 0; i < bases.size() && i < raws.size(); ++i) {
        const quint32 base = bases[i];
        if (m.addr < base)
            continue;
        const int end = int(m.addr - base) + a2lTypeSize(m.type);
        if (end > raws[i].size())
            continue;                       // a later block, or past this one
        return decode(raws[i], base, m, out);
    }
    return false;
}

int A2lModel::blockExtent(const QVector<A2lMeas> &meas, quint32 blockBase,
                          int limit)
{
    int extent = 0;

    for (const A2lMeas &m : meas) {
        if (m.addr < blockBase)
            continue;                       // belongs to a different block

        const int end = int(m.addr - blockBase) + a2lTypeSize(m.type);
        if (end > limit)
            continue;                       // past the next block, not ours
        if (end > extent)
            extent = end;
    }
    return extent;
}
