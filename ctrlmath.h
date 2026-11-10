#ifndef CTRLMATH_H
#define CTRLMATH_H

#include <QByteArray>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "a2lmodel.h"

// Pure helpers behind the flight-controller panel of the Attitude tab (no
// sockets, no widgets), so ctrl_selfcheck can test them. Everything that
// depends on the firmware layout is derived BY NAME from the A2L
// characteristic list; only the semantics documented in the A2L descriptions
// (status bits, yaw "-1 = keep heading", ARMED = EscMotorState 2) are encoded.

inline bool ctrlIsChar(const QString &name)
{
    return name.startsWith(QLatin1String("CAL_ctrl"));
}

// One SHORT_UPLOAD window: [lo, lo+len).
struct CtrlWindow {
    quint32 lo  = 0;
    int     len = 0;
};

// The window length of the CAL_ctrl* read-back. Deliberately different from the
// 60 B Calibration-tab chunk, the 56 B logging poll and the CAL_motor* window,
// so a reply is told from the others by (address, length).
constexpr int kCtrlWindowMax = 52;

// Greedy split of a set of characteristics (sorted by address) into windows of
// at most maxLen bytes that never cut a characteristic in two. Gaps between
// characteristics are read along (a window is contiguous).
inline QVector<CtrlWindow> ctrlSplitWindows(QVector<A2lChar> chars, int maxLen)
{
    QVector<CtrlWindow> out;
    std::sort(chars.begin(), chars.end(),
              [](const A2lChar &a, const A2lChar &b) { return a.addr < b.addr; });
    quint32 lo = 0, hi = 0;
    bool open = false;
    for (const A2lChar &c : chars) {
        const quint32 end = c.addr + quint32(a2lTypeSize(c.type));
        if (open && end - lo > quint32(maxLen)) {
            out.append({lo, int(hi - lo)});
            open = false;
        }
        if (!open) {
            lo = c.addr;
            hi = end;
            open = true;
        } else {
            hi = std::max(hi, end);
        }
    }
    if (open)
        out.append({lo, int(hi - lo)});
    return out;
}

// Windows covering every CAL_ctrl* characteristic.
inline QVector<CtrlWindow> ctrlReadWindows(const QVector<A2lChar> &chars)
{
    QVector<A2lChar> sel;
    for (const A2lChar &c : chars)
        if (ctrlIsChar(c.name))
            sel.append(c);
    return ctrlSplitWindows(sel, kCtrlWindowMax);
}

// Calibration-tab read plan: per 0x100 block one chunk list of <= maxLen
// bytes (the cal block grew from 84 to 148 bytes, so one SHORT_UPLOAD of 63 B
// no longer covers it). Same result as the loop it replaced in readCalibration().
inline QVector<CtrlWindow> calReadChunks(const QVector<A2lChar> &chars, int maxLen = 60)
{
    QVector<CtrlWindow> out;
    QVector<quint32> bases;
    for (const A2lChar &c : chars) {
        const quint32 b = c.addr & 0xFFFFFF00u;
        if (!bases.contains(b))
            bases.append(b);
    }
    std::sort(bases.begin(), bases.end());
    for (quint32 base : bases) {
        quint32 lo = 0xFFFFFFFFu, hi = 0;
        for (const A2lChar &c : chars) {
            if ((c.addr & 0xFFFFFF00u) != base)
                continue;
            lo = std::min(lo, c.addr);
            hi = std::max(hi, c.addr + quint32(a2lTypeSize(c.type)));
        }
        for (quint32 a = lo; a < hi; a += quint32(maxLen))
            out.append({a, int(std::min<quint32>(quint32(maxLen), hi - a))});
    }
    return out;
}

inline double ctrlDecodeChar(const A2lChar &c, const uchar *p)
{
    switch (c.type) {
    case A2lType::Float32: { float f; std::memcpy(&f, p, 4); return double(f); }
    case A2lType::Uint32:  return double(qFromLittleEndian<quint32>(p));
    case A2lType::Sint32:  return double(qFromLittleEndian<qint32>(p));
    case A2lType::Uint16:  return double(qFromLittleEndian<quint16>(p));
    case A2lType::Uint8:   return double(*p);
    }
    return qQNaN();
}

// Parse a reply: accepted only if (address, length) equals one of the windows
// exactly (a partial reply is rejected); every CAL_ctrl* char inside is decoded
// by its A2L offset and type.
inline QMap<QString, double> ctrlParseRead(const QVector<A2lChar> &chars,
                                           const QVector<CtrlWindow> &windows,
                                           quint32 address, const QByteArray &data)
{
    QMap<QString, double> values;
    bool known = false;
    for (const CtrlWindow &w : windows)
        if (w.lo == address && w.len == data.size())
            known = true;
    if (!known)
        return values;
    for (const A2lChar &c : chars) {
        if (!ctrlIsChar(c.name) || c.addr < address)
            continue;
        const int off = int(c.addr - address);
        if (off + a2lTypeSize(c.type) > data.size())
            continue;
        values.insert(c.name, ctrlDecodeChar(c, reinterpret_cast<const uchar *>(data.constData()) + off));
    }
    return values;
}

// ---- semantics -----------------------------------------------------------

constexpr int kCtrlArmedState = 2;          // EscMotorState: 0 disarmed, 1 init, 2 armed, 3 failsafe

// CtrlStatus bits (A2L description).
constexpr quint32 kCtrlStDriving = 1u;
constexpr quint32 kCtrlStNavInvalid = 2u;
constexpr quint32 kCtrlStTilt = 4u;

inline QString ctrlStatusText(quint32 s)
{
    QStringList parts;
    if (s & kCtrlStDriving)    parts << QStringLiteral("driving");
    if (s & kCtrlStNavInvalid) parts << QStringLiteral("navigation estimate invalid");
    if (s & kCtrlStTilt)       parts << QStringLiteral("tilt limit exceeded");
    const quint32 other = s & ~(kCtrlStDriving | kCtrlStNavInvalid | kCtrlStTilt);
    if (other)
        parts << QStringLiteral("unknown bits 0x%1").arg(other, 0, 16);
    return parts.isEmpty() ? QStringLiteral("idle (shadow)") : parts.join(QStringLiteral(", "));
}

// Why the firmware dropped the controller, from the fault bits seen last. 0 =
// neither bit was ever seen: the A2L's third cause (flight core silent / request
// unusable) cannot be told from here.
inline QString ctrlTripReason(quint32 faultBits)
{
    QStringList parts;
    if (faultBits & kCtrlStNavInvalid) parts << QStringLiteral("navigation estimate invalid");
    if (faultBits & kCtrlStTilt)       parts << QStringLiteral("tilt limit exceeded");
    return parts.isEmpty() ? QStringLiteral("request unusable (flight core silent) or no fault bit caught between two samples")
                           : parts.join(QStringLiteral(" + "));
}

// Yaw word: 0..360 = heading to hold, anything else = keep the current heading.
inline bool ctrlYawIsHeading(double v) { return std::isfinite(v) && v >= 0.0 && v <= 360.0; }

// Write gates. NaN fails every comparison, so test for finiteness explicitly.
inline bool ctrlTiltWritable(double v, double tiltMax)
{
    return std::isfinite(v) && std::isfinite(tiltMax) && std::fabs(v) <= tiltMax;
}
inline bool ctrlYawWritable(double v) { return ctrlYawIsHeading(v); }
inline bool ctrlThrustWritable(double v, double hi)
{
    return std::isfinite(v) && v >= 0.0 && v <= hi;
}

// Fold an angle difference into [-180, 180).
inline double ctrlWrap180(double d)
{
    if (!std::isfinite(d))
        return d;
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

// Read-back equals what was requested (the firmware stores float32; a clamp or a
// reset shows up as a difference).
inline bool ctrlSameValue(double requested, double readBack)
{
    return std::isfinite(requested) && std::isfinite(readBack)
        && std::fabs(requested - readBack) <= 1e-3 + 1e-6 * std::fabs(requested);
}

// Running unwrap for the yaw plot: next unwrapped value after raw sample `raw`,
// given the previous raw and unwrapped values.
inline double ctrlUnwrapNext(double prevRaw, double prevUnwrapped, double raw)
{
    return prevUnwrapped + ctrlWrap180(raw - prevRaw);
}

// Returns true if everything holds; appends one line per case to *report.
inline bool ctrlSelfCheck(QString *report)
{
    bool all = true;
    auto check = [&](const char *name, bool ok) {
        if (report)
            report->append(QStringLiteral("[%1] %2\n").arg(ok ? "PASS" : "FAIL", QString::fromLatin1(name)));
        all = all && ok;
    };

    // Layout of the real A2L (Xcp_Cal, 0x70030154..0x70030194).
    QVector<A2lChar> chars;
    auto add = [&](const char *n, quint32 a, A2lType t) {
        A2lChar c; c.name = n; c.addr = a; c.type = t; chars.append(c);
    };
    add("CAL_motorCmd", 0x70030140, A2lType::Uint32);
    add("CAL_motorSpMode", 0x70030152, A2lType::Uint16);
    add("CAL_ctrlMode", 0x70030154, A2lType::Uint32);
    add("CAL_ctrlThrustN", 0x70030158, A2lType::Float32);
    add("CAL_ctrlTiltMaxDeg", 0x7003015C, A2lType::Float32);
    add("CAL_ctrlRpmMin", 0x70030160, A2lType::Float32);
    const char *gains[] = { "CAL_ctrlOmKp_", "CAL_ctrlTauKp_", "CAL_ctrlTauKi_" };
    quint32 a = 0x70030164;
    for (const char *g : gains)
        for (int i = 0; i < 3; ++i, a += 4)
            add(qPrintable(QStringLiteral("%1%2").arg(g).arg(i)), a, A2lType::Float32);
    for (int i = 0; i < 3; ++i, a += 4)
        add(qPrintable(QStringLiteral("CAL_ctrlAttSp_%1").arg(i)), a, A2lType::Float32);

    const QVector<CtrlWindow> w = ctrlReadWindows(chars);
    check("ctrl windows: two, 0x70030154 len 52 + 0x70030188 len 12",
          w.size() == 2 && w[0].lo == 0x70030154u && w[0].len == 52 && w[1].lo == 0x70030188u && w[1].len == 12);
    bool within = true;
    for (const CtrlWindow &x : w) within = within && x.len <= 60;
    check("ctrl windows: each below the 63 B SHORT_UPLOAD cap", within);

    // A reply: window 2 = roll 5.5, pitch -3.25, yaw -1.
    QByteArray reply(12, '\0');
    const float sp[3] = { 5.5f, -3.25f, -1.0f };
    std::memcpy(reply.data(), sp, 12);
    QMap<QString, double> v = ctrlParseRead(chars, w, 0x70030188u, reply);
    check("parse: setpoints decoded by name",
          v.size() == 3 && v["CAL_ctrlAttSp_0"] == 5.5 && v["CAL_ctrlAttSp_1"] == -3.25 && v["CAL_ctrlAttSp_2"] == -1.0);
    check("parse: wrong length rejected", ctrlParseRead(chars, w, 0x70030188u, reply.left(8)).isEmpty());
    check("parse: unknown address rejected", ctrlParseRead(chars, w, 0x70030140u, reply).isEmpty());

    QByteArray head(52, '\0');
    const quint32 mode = 1;
    qToLittleEndian<quint32>(mode, head.data());
    const float thrust = 12.5f;
    std::memcpy(head.data() + 4, &thrust, 4);
    v = ctrlParseRead(chars, w, 0x70030154u, head);
    check("parse: mode as integer, thrust as float, 13 words",
          v.size() == 13 && v["CAL_ctrlMode"] == 1.0 && v["CAL_ctrlThrustN"] == 12.5);

    // Calibration chunks: block 0x70030100 .. 0x70030194 = 148 B -> three <=60 B chunks, none cut.
    QVector<A2lChar> cal = chars;
    A2lChar first; first.name = "CAL_fsVext"; first.addr = 0x7003013C; first.type = A2lType::Float32; cal.append(first);
    const QVector<CtrlWindow> ch = calReadChunks(cal);
    bool covered = true;
    quint32 end = 0;
    for (const A2lChar &c : cal) {
        bool in = false;
        for (const CtrlWindow &x : ch)
            if (c.addr >= x.lo && c.addr + quint32(a2lTypeSize(c.type)) <= x.lo + quint32(x.len)) in = true;
        covered = covered && in;
        end = std::max(end, c.addr + quint32(a2lTypeSize(c.type)));
    }
    check("cal chunks: every characteristic inside one chunk, 0x70030194 reached", covered && end == 0x70030194u);
    bool small = true;
    for (const CtrlWindow &x : ch) small = small && x.len <= 60;
    check("cal chunks: none above 60 B", small);

    check("status 0 -> idle", ctrlStatusText(0) == "idle (shadow)");
    check("status 1 -> driving", ctrlStatusText(1) == "driving");
    check("status 3 -> driving + nav invalid", ctrlStatusText(3) == "driving, navigation estimate invalid");
    check("status 4 -> tilt", ctrlStatusText(4) == "tilt limit exceeded");
    check("status 8 -> unknown bit shown", ctrlStatusText(8).contains("0x8"));
    check("trip reason: nav + tilt", ctrlTripReason(6) == "navigation estimate invalid + tilt limit exceeded");
    check("trip reason: none seen says so", ctrlTripReason(0).contains("unusable"));

    check("yaw: 0 / 359.9 / 360 are headings", ctrlYawIsHeading(0) && ctrlYawIsHeading(359.9) && ctrlYawIsHeading(360));
    check("yaw: -1 / 361 / NaN are not", !ctrlYawIsHeading(-1) && !ctrlYawIsHeading(361) && !ctrlYawIsHeading(qQNaN()));
    check("tilt gate: 30 passes at 30, 30.1 / NaN / inf do not",
          ctrlTiltWritable(30, 30) && ctrlTiltWritable(-30, 30) && !ctrlTiltWritable(30.1, 30)
          && !ctrlTiltWritable(qQNaN(), 30) && !ctrlTiltWritable(5, qQNaN()) && !ctrlTiltWritable(qInf(), 90));
    check("thrust gate: 0 / 40 pass, -0.1 / 40.1 / NaN do not",
          ctrlThrustWritable(0, 40) && ctrlThrustWritable(40, 40) && !ctrlThrustWritable(-0.1, 40)
          && !ctrlThrustWritable(40.1, 40) && !ctrlThrustWritable(qQNaN(), 40));
    check("wrap180: 190 -> -170, -190 -> 170, 359 -> -1, 180 -> -180",
          std::fabs(ctrlWrap180(190) + 170) < 1e-9 && std::fabs(ctrlWrap180(-190) - 170) < 1e-9
          && std::fabs(ctrlWrap180(359) + 1) < 1e-9 && std::fabs(ctrlWrap180(180) + 180) < 1e-9);
    check("same value: float32 5.1 vs 5.1, clamp 30 vs 45 differ, NaN never equal",
          ctrlSameValue(5.1, double(5.1f)) && !ctrlSameValue(45, 30) && !ctrlSameValue(qQNaN(), qQNaN()));
    // yaw unwrap across the +-180 seam stays continuous
    double raw = 170, un = 170;
    bool cont = true;
    for (double r : { 175.0, 179.0, -178.0, -170.0, -175.0, 178.0 }) {
        const double nu = ctrlUnwrapNext(raw, un, r);
        cont = cont && std::fabs(nu - un) < 10.0;
        raw = r; un = nu;
    }
    check("yaw unwrap continuous across the seam", cont && std::fabs(un - 178.0 - 360.0 * std::round((un - 178.0) / 360.0)) < 1e-9);
    return all;
}

#endif // CTRLMATH_H
