#ifndef MOTORPROTO_H
#define MOTORPROTO_H

#include <QByteArray>
#include <QQueue>
#include <QMap>
#include <QSet>
#include <QString>
#include <QVector>
#include <QtEndian>
#include "a2lmodel.h"

// Pure helpers behind XcpPanel's Motors-tab plumbing (no sockets, no widgets),
// so xcp_queue_selfcheck can test them: which addresses belong to the CAL_motor*
// words, the one-SHORT_UPLOAD read-back window, and the reply parser. Everything
// is derived BY NAME from the A2L characteristic list.

inline bool motorIsMotorChar(const QString &name)
{
    return name.startsWith(QLatin1String("CAL_motor"));
}

inline bool motorIsSetpointChar(const QString &name)
{
    return name.startsWith(QLatin1String("CAL_motorManualSp_"));
}

// Addresses of every CAL_motor* word (cmd, manual, setpoints), or of the
// setpoints only. DISARM purges the former so no queued ARM / manual-ON write
// can execute after the kill.
inline QSet<quint32> motorWriteAddrs(const QVector<A2lChar> &chars, bool setpointsOnly)
{
    QSet<quint32> a;
    for (const A2lChar &c : chars)
        if (setpointsOnly ? motorIsSetpointChar(c.name) : motorIsMotorChar(c.name))
            a.insert(c.addr);
    return a;
}

// The single SHORT_UPLOAD window covering all CAL_motor* words.
struct MotorReadPlan {
    bool    ok  = false;
    quint32 lo  = 0;
    int     len = 0;
    QString reason;     // why !ok (empty if there is simply nothing to read)
};

inline MotorReadPlan motorReadPlan(const QVector<A2lChar> &chars)
{
    MotorReadPlan p;
    quint32 lo = 0xFFFFFFFFu, hi = 0;
    for (const A2lChar &c : chars) {
        if (!motorIsMotorChar(c.name))
            continue;
        lo = qMin(lo, c.addr);
        hi = qMax(hi, c.addr + quint32(a2lTypeSize(c.type)));
    }
    if (hi <= lo)
        return p;
    const quint32 span = hi - lo;
    if (span > 63u) {
        p.reason = QStringLiteral("CAL_motor* span %1 bytes exceeds one SHORT_UPLOAD - read-back disabled").arg(span);
        return p;
    }
    p.ok = true;
    p.lo = lo;
    p.len = int(span);
    return p;
}

// Parse a reply to the plan's read into name -> value. Only an exact-length
// reply at the plan's address is accepted (partial replies are rejected);
// offsets come from the A2L addresses, UWORD and ULONG are read by their size.
inline QMap<QString, quint32> motorParseRead(const QVector<A2lChar> &chars, const MotorReadPlan &plan,
                                             quint32 address, const QByteArray &data)
{
    QMap<QString, quint32> values;
    if (!plan.ok || address != plan.lo || data.size() != plan.len)
        return values;
    for (const A2lChar &c : chars) {
        if (!motorIsMotorChar(c.name) || c.addr < address)
            continue;
        const int off = int(c.addr - address);
        const int sz = a2lTypeSize(c.type);
        if (off + sz > data.size())
            continue;
        const uchar *p = reinterpret_cast<const uchar *>(data.constData()) + off;
        if (sz == 2)
            values.insert(c.name, qFromLittleEndian<quint16>(p));
        else if (sz == 4)
            values.insert(c.name, qFromLittleEndian<quint32>(p));
    }
    return values;
}

// Bookkeeping for read-backs in flight. Each read is tagged with the
// CAL_motor* write sequence number at the moment it was issued; replies arrive
// in issue order, so take() yields the tag of the request a reply belongs to.
// INVARIANT: a reply is fresh only if no CAL_motor* write was enqueued after
// its own request. When full, issue() REFUSES a new read (it never evicts or
// shifts tags), so a reply can never pick up a newer request's tag.
class MotorReadTags
{
public:
    static constexpr int kMax = 8;

    bool issue(quint32 writeSeq)
    {
        if (m_q.size() >= kMax)
            return false;
        m_q.enqueue(writeSeq);
        return true;
    }
    bool pending() const { return !m_q.isEmpty(); }
    int  size() const { return m_q.size(); }
    // Tag of the oldest outstanding read (the one the next reply / error is for).
    bool take(quint32 *tag)
    {
        if (m_q.isEmpty())
            return false;
        *tag = m_q.dequeue();
        return true;
    }
    void dropOldest() { if (!m_q.isEmpty()) m_q.dequeue(); }   // its request errored
    void clear() { m_q.clear(); }                               // disconnect
    static bool isFresh(quint32 tag, quint32 currentWriteSeq) { return tag == currentWriteSeq; }

private:
    QQueue<quint32> m_q;
};

#endif // MOTORPROTO_H
