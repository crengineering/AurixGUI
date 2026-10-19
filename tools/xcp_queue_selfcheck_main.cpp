// Headless self-check of the XCP transport queue (Coalesce / Priority /
// purgeWrites, the DISARM purge sequence) and of the Motors read-back parser.
// XcpClient is driven through its private queue by the friend
// XcpClientTestAccess, with the "in flight" slot occupied so nothing is ever
// sent: no socket traffic, no board. Built and run by build.bat and CI.
#include "xcpclient.h"
#include "motorproto.h"

#include <QCoreApplication>
#include <cstdio>

struct XcpClientTestAccess {
    using Req = XcpClient::Request;
    using Type = XcpClient::ReqType;

    static void arm(XcpClient &c, quint32 inflightAddr = 0xF0, Type t = Type::MemRead)
    {
        c.m_connected = true;
        c.m_busy = true;                       // something is "in flight": enqueue() never sends
        c.m_current = Req{ t, QByteArray("x"), inflightAddr };
    }
    static void push(XcpClient &c, Type t, quint32 addr)
    {
        c.m_queue.enqueue(Req{ t, QByteArray(), addr });
    }
    static const QQueue<Req> &queue(XcpClient &c) { return c.m_queue; }
    static int payload(const Req &r) { return quint8(r.packet.at(8)); }
    static quint32 inflightAddr(XcpClient &c) { return c.m_current.addr; }
    static void setConnected(XcpClient &c, bool on) { c.m_connected = on; }
};
using T = XcpClientTestAccess;

namespace {

int g_fail = 0;
void check(const char *name, bool ok)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

QByteArray w16(quint16 v) { QByteArray b(2, '\0'); qToLittleEndian<quint16>(v, b.data()); return b; }
QByteArray w32(quint32 v) { QByteArray b(4, '\0'); qToLittleEndian<quint32>(v, b.data()); return b; }

QVector<A2lChar> motorChars()
{
    QVector<A2lChar> v;
    auto add = [&](const char *n, quint32 a, A2lType t) { A2lChar c; c.name = n; c.addr = a; c.type = t; v.append(c); };
    add("CAL_motorManualSp_M2", 0x2142 + 8, A2lType::Uint16);   // deliberately out of order
    add("CAL_motorCmd", 0x2140, A2lType::Uint32);
    add("CAL_motorManual", 0x2144, A2lType::Uint32);
    add("CAL_motorManualSp_M1", 0x2148, A2lType::Uint16);
    add("CAL_motorManualSp_M4", 0x214E, A2lType::Uint16);
    add("CAL_motorManualSp_M3", 0x214C, A2lType::Uint16);
    add("CAL_other", 0x2000, A2lType::Uint32);
    return v;
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    using Type = XcpClientTestAccess::Type;

    // not connected / oversize: nothing enqueued, false returned
    {
        XcpClient c;
        check("write while disconnected returns false", !c.writeMemory(0x10, w16(1)));
        T::arm(c);
        check("oversize write returns false", !c.writeMemory(0x10, QByteArray(57, 'x')));
        check("normal write returns true", c.writeMemory(0x10, w16(1)));
    }

    // Coalesce: replaces only a queued MemWrite at the same address, keeps its slot
    {
        XcpClient c; T::arm(c);
        c.writeMemory(0x10, QByteArray(1, char(1)));
        T::push(c, Type::GetStatus, 0);
        c.writeMemory(0x20, QByteArray(1, char(2)));
        T::push(c, Type::MemRead, 0x40);
        c.writeMemory(0x10, QByteArray(1, char(9)), XcpClient::WriteMode::Coalesce);
        check("coalesce: queue did not grow", T::queue(c).size() == 4);
        check("coalesce: payload replaced in place (slot 0)", T::queue(c)[0].addr == 0x10 && T::payload(T::queue(c)[0]) == 9);
        check("coalesce: neighbours untouched", T::queue(c)[2].addr == 0x20 && T::payload(T::queue(c)[2]) == 2);
        c.writeMemory(0x40, QByteArray(1, char(5)), XcpClient::WriteMode::Coalesce);
        check("coalesce: a queued MemRead at the same address is not replaced (appended)",
              T::queue(c).size() == 5 && T::queue(c)[3].type == Type::MemRead && T::queue(c)[4].type == Type::MemWrite);
        c.writeMemory(0x50, QByteArray(1, char(5)), XcpClient::WriteMode::Coalesce);
        check("coalesce: no queued write at that address -> appended", T::queue(c).size() == 6 && T::queue(c)[5].addr == 0x50);
        check("coalesce: in-flight request untouched", T::inflightAddr(c) == 0xF0);
    }

    // Priority: behind the in-flight request, ahead of everything queued
    {
        XcpClient c; T::arm(c);
        T::push(c, Type::GetStatus, 0);
        c.writeMemory(0x10, QByteArray(1, char(1)));
        c.writeMemory(0x30, QByteArray(1, char(0)), XcpClient::WriteMode::Priority);
        check("priority: head of the queue, ahead of GET_STATUS",
              T::queue(c).size() == 3 && T::queue(c)[0].type == Type::MemWrite && T::queue(c)[0].addr == 0x30
                  && T::queue(c)[1].type == Type::GetStatus);
        check("priority: in-flight request still the old one", T::inflightAddr(c) == 0xF0);
    }

    // purgeWrites: exactly the given addresses, queued only
    {
        XcpClient c; T::arm(c, 0x1, Type::MemWrite);   // in flight: a write to 0x1
        T::push(c, Type::MemWrite, 0x1);
        T::push(c, Type::MemWrite, 0x2);
        T::push(c, Type::MemRead, 0x2);
        T::push(c, Type::MemWrite, 0x3);
        c.purgeWrites({ 0x1, 0x2 });
        check("purge: removed queued writes at the given addresses only",
              T::queue(c).size() == 2 && T::queue(c)[0].type == Type::MemRead && T::queue(c)[1].addr == 0x3);
        check("purge: in-flight write untouched", T::inflightAddr(c) == 0x1);
    }

    // DISARM sequence: no CAL_motor* write left except the one CAL_motorCmd=0 at the head
    {
        const QVector<A2lChar> chars = motorChars();
        XcpClient c; T::arm(c);
        c.writeMemory(0x2000, w32(7));                         // unrelated cal write
        c.writeMemory(0x2140, w32(1));                         // queued ARM
        c.writeMemory(0x2144, w32(1));                         // queued manual ON
        c.writeMemory(0x2148, w16(500), XcpClient::WriteMode::Coalesce);
        T::push(c, Type::GetStatus, 0);
        c.purgeWrites(motorWriteAddrs(chars, false));
        c.writeMemory(0x2140, w32(0), XcpClient::WriteMode::Priority);
        int motorWrites = 0; bool armLeft = false;
        const QSet<quint32> motor = motorWriteAddrs(chars, false);
        for (const auto &r : T::queue(c))
            if (r.type == Type::MemWrite && motor.contains(r.addr)) {
                ++motorWrites;
                if (r.addr == 0x2140 && T::payload(r) != 0) armLeft = true;
            }
        check("DISARM: exactly one CAL_motor* write queued", motorWrites == 1);
        check("DISARM: no ARM (cmd=1) left behind the kill", !armLeft);
        check("DISARM: the cmd=0 write is at the head", T::queue(c)[0].addr == 0x2140 && T::payload(T::queue(c)[0]) == 0);
        check("DISARM: unrelated write and GET_STATUS survive", T::queue(c).size() == 3);
    }

    // address sets and read plan
    {
        const QVector<A2lChar> chars = motorChars();
        check("motorWriteAddrs: all CAL_motor* = 6", motorWriteAddrs(chars, false).size() == 6);
        check("motorWriteAddrs: setpoints only = 4", motorWriteAddrs(chars, true).size() == 4);
        const MotorReadPlan p = motorReadPlan(chars);
        check("read plan: lo 0x2140, 16 bytes", p.ok && p.lo == 0x2140 && p.len == 16);

        QByteArray d(16, '\0');
        d.replace(0, 4, w32(1));
        d.replace(4, 4, w32(1));
        d.replace(8, 2, w16(100));    // M1
        d.replace(10, 2, w16(200));   // M2
        d.replace(12, 2, w16(300));   // M3
        d.replace(14, 2, w16(400));   // M4
        const auto v = motorParseRead(chars, p, 0x2140, d);
        check("parse: ULONG cmd/manual", v.value("CAL_motorCmd") == 1 && v.value("CAL_motorManual") == 1);
        check("parse: UWORD setpoints at their A2L offsets, any list order",
              v.value("CAL_motorManualSp_M1") == 100 && v.value("CAL_motorManualSp_M2") == 200
                  && v.value("CAL_motorManualSp_M3") == 300 && v.value("CAL_motorManualSp_M4") == 400);
        check("parse: only CAL_motor* words returned", v.size() == 6);
        check("parse: partial reply (15 bytes) rejected", motorParseRead(chars, p, 0x2140, d.left(15)).isEmpty());
        check("parse: over-long reply rejected", motorParseRead(chars, p, 0x2140, d + QByteArray(1, 'x')).isEmpty());
        check("parse: wrong address rejected", motorParseRead(chars, p, 0x2141, d).isEmpty());

        // CAL_motorSpMax (0x2150 here, after M4 and a reserved word) joins the read window
        // but not the DISARM purge set.
        QVector<A2lChar> withCeil = chars;
        A2lChar sm; sm.name = "CAL_motorSpMax"; sm.addr = 0x214E + 2 + 2; sm.type = A2lType::Uint16; withCeil.append(sm);
        const MotorReadPlan pc = motorReadPlan(withCeil);
        check("ceiling: read plan widens to 20 bytes", pc.ok && pc.lo == 0x2140 && pc.len == 20);
        check("ceiling: not in the DISARM purge set (6 stay), not a setpoint",
              motorWriteAddrs(withCeil, false).size() == 6 && motorWriteAddrs(withCeil, true).size() == 4);
        QByteArray dc(20, '\0');
        dc.replace(18, 2, w16(1600));
        check("ceiling: parsed from the read-back", motorParseRead(withCeil, pc, 0x2140, dc).value("CAL_motorSpMax") == 1600);

        QVector<A2lChar> far = chars;
        A2lChar f; f.name = "CAL_motorManualSp_M9"; f.addr = 0x2140 + 80; f.type = A2lType::Uint16; far.append(f);
        const MotorReadPlan bad = motorReadPlan(far);
        check("read plan: span > 63 bytes -> not ok, reason names it",
              !bad.ok && bad.reason.contains("exceeds one SHORT_UPLOAD"));
        check("parse with a bad plan returns nothing", motorParseRead(far, bad, 0x2140, d).isEmpty());
    }

    // read-tag bookkeeping: never fresh unless issued after the last write
    {
        MotorReadTags t;
        quint32 seq = 0, tag = 0;
        for (int i = 0; i < MotorReadTags::kMax; ++i)
            t.issue(seq);                                   // 8 unanswered reads, all at seq 0
        ++seq;                                              // a CAL_motor* write follows
        check("tags: 9th read is refused, nothing evicted", !t.issue(seq) && t.size() == MotorReadTags::kMax);
        bool anyFresh = false;
        while (t.take(&tag))
            anyFresh = anyFresh || MotorReadTags::isFresh(tag, seq);
        check("tags: replies to pre-write reads are never fresh (even after overflow)", !anyFresh);
        check("tags: a read issued after the write is fresh", t.issue(seq) && t.take(&tag) && MotorReadTags::isFresh(tag, seq));
        t.issue(1); t.issue(2);
        t.dropOldest();
        check("tags: errored read drops the oldest tag", t.take(&tag) && tag == 2);
        t.issue(3); t.clear();
        check("tags: clear() (disconnect) empties the queue", !t.pending());
    }

    std::printf("%s\n", g_fail == 0 ? "[xcp_queue_selfcheck] PASS" : "[xcp_queue_selfcheck] FAIL");
    return g_fail == 0 ? 0 : 1;
}
