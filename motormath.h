#ifndef MOTORMATH_H
#define MOTORMATH_H

#include <QString>
#include <QtGlobal>

// Pure logic behind the "Motors" tab: slider <-> DShot mapping, throttle-bar
// scaling and the EscMotorState text. Header-only and free of QWidget / A2L /
// XCP so motor_selfcheck can link it alone (same idea as positionmath.h).
//
// DShot (docs/DSHOT.md): 0 = motor stop, 1..47 = special commands (beeps,
// direction changes...), 48..2047 = throttle. The GUI must NEVER put 1..47 on
// the wire; the slider therefore has one extra "STOP" position below 48.

inline constexpr int kMotorSliderStop = 47;     // slider position meaning STOP
inline constexpr int kMotorDshotMin   = 48;     // lowest throttle value
inline constexpr int kMotorDshotMax   = 2047;   // full throttle

// Slider position (47..2047) -> value for CAL_motorManualSp_Mn: 0 or 48..max.
inline int motorSliderToDshot(int pos, int maxDshot = kMotorDshotMax)
{
    if (pos <= kMotorSliderStop)
        return 0;
    return qBound(kMotorDshotMin, pos, maxDshot);
}

// Inverse: 0 (and anything that is not a throttle value) -> STOP position.
inline int motorDshotToSlider(int dshot, int maxDshot = kMotorDshotMax)
{
    if (dshot < kMotorDshotMin)
        return kMotorSliderStop;
    return qMin(dshot, maxDshot);
}

// Values the GUI is allowed to write to a manual setpoint.
inline bool motorDshotWritable(quint32 v, int maxDshot = kMotorDshotMax)
{
    return v == 0 || (v >= quint32(kMotorDshotMin) && v <= quint32(maxDshot));
}

// DShot throttle ceiling (CAL_motorSpMax, firmware RAM calibration, default
// 1600): valid range is 48..2047; the firmware reverts anything else.
inline bool motorCeilingValid(quint32 v)
{
    return v >= quint32(kMotorDshotMin) && v <= quint32(kMotorDshotMax);
}

// Upper end of the slider / spinbox range. spHi = A2L upper limit of
// CAL_motorManualSp_Mn; ceiling = the firmware's read-back CAL_motorSpMax, or
// -1 when unknown / not in the A2L (then the fixed A2L range applies). The
// STOP position (47) and the 48..max encoding never change, only the top.
inline int motorEffectiveMax(int spHi, int ceiling)
{
    int m = qBound(kMotorDshotMin, spHi, kMotorDshotMax);
    if (ceiling >= kMotorDshotMin)
        m = qMin(m, qMin(ceiling, kMotorDshotMax));
    return m;
}

// Speed mode (CAL_motorSpMode = 1): CAL_motorManualSp_Mn is mechanical rpm and
// the firmware maps it to DShot through the thrust-rig run 5 table in motor.c
// (Motor_convert_speed_to_throttle). Mirrored here ONLY to show the operator
// which DShot value the firmware will command and to size the rpm slider; the
// firmware stays the authority and nothing is written from this table.
// Linear interpolation, uint32 integer truncation, above the last breakpoint
// the last value (1600). The firmware's ceiling clamps afterwards.
inline constexpr int kMotorRpmMax = 8257;      // last table breakpoint

inline int motorSpeedToDshot(quint32 rpm)
{
    static constexpr quint32 kX[] = { 0, 1, 129, 1300, 2629, 3829, 4929, 5929, 6814, 8257 };
    static constexpr quint32 kY[] = { 0, 48, 48, 200, 400, 600, 800, 1000, 1200, 1600 };
    constexpr int n = int(sizeof(kX) / sizeof(kX[0]));
    if (rpm >= kX[n - 1])
        return int(kY[n - 1]);
    for (int i = 1; i < n; ++i)
        if (rpm <= kX[i])
            return int(kY[i - 1] + (rpm - kX[i - 1]) * (kY[i] - kY[i - 1]) / (kX[i] - kX[i - 1]));
    return int(kY[n - 1]);
}

// Largest rpm whose table value does not exceed maxDshot (the ceiling in
// force); kMotorRpmMax when the ceiling is not below the table's top.
inline int motorRpmMaxForDshot(int maxDshot)
{
    if (maxDshot >= motorSpeedToDshot(kMotorRpmMax))
        return kMotorRpmMax;
    int lo = 0, hi = kMotorRpmMax;              // f(lo) <= maxDshot < f(hi); f is monotonic
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (motorSpeedToDshot(quint32(mid)) <= maxDshot) lo = mid; else hi = mid;
    }
    return lo;
}

// DShot value the firmware commands for a speed setpoint: table, then ceiling.
inline int motorSpeedCommand(quint32 rpm, int ceiling)
{
    const int d = motorSpeedToDshot(rpm);
    return (ceiling >= kMotorDshotMin && d > ceiling) ? ceiling : d;
}

// CAL_motorSpMode read-back -> 0 throttle / 1 speed. The firmware treats any
// nonzero value as SPEED and does not validate it, so neither do we.
inline int motorModeFromRead(quint32 v) { return v != 0 ? 1 : 0; }

// Write gate for CAL_motorManualSp_Mn in the unit the firmware will read it:
// throttle mode keeps the DShot rule (0, or 48..max; 1..47 are commands),
// speed mode is 0..kMotorRpmMax rpm (the firmware maps 1..47 rpm to >= 48 and
// clamps above the table itself). 2026-11-01: the DShot rule applied to rpm
// silently dropped every request above 2047 rpm (DShot 312 at most).
inline bool motorSetpointWritable(quint32 v, bool speedMode, int maxDshot = kMotorDshotMax)
{
    return speedMode ? v <= quint32(kMotorRpmMax) : motorDshotWritable(v, maxDshot);
}

// EscMnDshotCmd -> 0..1 bar fill (fraction of 2047). Garbage clamps, never NaN.
inline double motorBarFraction(double dshot)
{
    if (!(dshot > 0.0))          // also catches NaN
        return 0.0;
    return qMin(dshot / double(kMotorDshotMax), 1.0);
}

inline QString motorStateName(int state)
{
    switch (state) {
    case 0: return QStringLiteral("DISARMED");
    case 1: return QStringLiteral("INIT");
    case 2: return QStringLiteral("ARMED");
    case 3: return QStringLiteral("FAILSAFE");
    default: return QStringLiteral("--");
    }
}

// Returns true if everything holds; appends one line per case to *report.
inline bool motorSelfCheck(QString *report)
{
    bool all = true;
    auto check = [&](const char *name, bool ok) {
        if (report)
            report->append(QStringLiteral("[%1] %2\n")
                               .arg(ok ? "PASS" : "FAIL", QString::fromLatin1(name)));
        all = all && ok;
    };

    check("47 -> 0 (STOP)", motorSliderToDshot(47) == 0);
    check("48 -> 48", motorSliderToDshot(48) == 48);
    check("2047 -> 2047", motorSliderToDshot(2047) == 2047);
    check("A2L max 1000 clamps 2047 -> 1000", motorSliderToDshot(2047, 1000) == 1000);

    bool never1to47 = true, writable = true, roundTrip = true;
    for (int p = kMotorSliderStop; p <= kMotorDshotMax; ++p) {
        const int d = motorSliderToDshot(p);
        if (d >= 1 && d <= 47) never1to47 = false;
        if (!motorDshotWritable(quint32(d))) writable = false;
        if (p >= kMotorDshotMin && motorDshotToSlider(d) != p) roundTrip = false;
    }
    check("no slider position yields DShot 1..47", never1to47);
    check("every slider position is writable", writable);
    check("slider<->dshot round trip 48..2047", roundTrip);
    check("dshot 0 -> STOP position", motorDshotToSlider(0) == kMotorSliderStop);
    check("dshot 20 (command) -> STOP position", motorDshotToSlider(20) == kMotorSliderStop);
    check("writable rejects 1, 47, 2048",
          !motorDshotWritable(1) && !motorDshotWritable(47) && !motorDshotWritable(2048));
    check("writable accepts 0, 48, 2047",
          motorDshotWritable(0) && motorDshotWritable(48) && motorDshotWritable(2047));

    check("ceiling valid 48 / 1600 / 2047", motorCeilingValid(48) && motorCeilingValid(1600) && motorCeilingValid(2047));
    check("ceiling invalid 0 / 47 / 2048", !motorCeilingValid(0) && !motorCeilingValid(47) && !motorCeilingValid(2048));
    check("effective max: ceiling 1600 -> 1600", motorEffectiveMax(2047, 1600) == 1600);
    check("effective max: ceiling 2047 -> 2047", motorEffectiveMax(2047, 2047) == 2047);
    check("effective max: no ceiling (fallback) -> 2047", motorEffectiveMax(2047, -1) == 2047);
    check("effective max: A2L limit 1000 wins over ceiling 1600", motorEffectiveMax(1000, 1600) == 1000);
    check("effective max: ceiling 48 -> 48", motorEffectiveMax(2047, 48) == 48);
    check("1600 ceiling: slider 2047 writes 1600, 47 still STOP",
          motorSliderToDshot(2047, 1600) == 1600 && motorSliderToDshot(47, 1600) == 0);
    check("1600 ceiling: 1601 not writable, 1600 / 0 are",
          !motorDshotWritable(1601, 1600) && motorDshotWritable(1600, 1600) && motorDshotWritable(0, 1600));

    check("speed table: 0 -> 0, 1 -> 48", motorSpeedToDshot(0) == 0 && motorSpeedToDshot(1) == 48);
    check("speed table: 128 -> 48, 129 -> 48, 650 -> 115, 1299 -> 199, 1300 -> 200 (run 6: breakpoint 129 rpm)",
          motorSpeedToDshot(128) == 48 && motorSpeedToDshot(129) == 48 && motorSpeedToDshot(650) == 115 && motorSpeedToDshot(1299) == 199 && motorSpeedToDshot(1300) == 200);
    check("speed table: 1964 -> 299, 4929 -> 800", motorSpeedToDshot(1964) == 299 && motorSpeedToDshot(4929) == 800);
    check("speed table: 8257 -> 1600, 9000 -> 1600", motorSpeedToDshot(8257) == 1600 && motorSpeedToDshot(9000) == 1600);
    bool mono = true;
    for (quint32 r = 1; r <= 9000; ++r)
        if (motorSpeedToDshot(r) < motorSpeedToDshot(r - 1) || (r <= 8257 && motorSpeedToDshot(r) < 48)) mono = false;
    check("speed table: monotonic, never 1..47", mono);
    check("rpm max: ceiling 1600 -> 8257, 2047 -> 8257, 1000 -> 5933",
          motorRpmMaxForDshot(1600) == 8257 && motorRpmMaxForDshot(2047) == 8257 && motorRpmMaxForDshot(1000) == 5933);
    bool inv = true;
    for (int c : { 48, 49, 123, 200, 799, 1000, 1599 }) {
        const int m = motorRpmMaxForDshot(c);
        if (motorSpeedToDshot(quint32(m)) > c || motorSpeedToDshot(quint32(m + 1)) <= c) inv = false;
    }
    check("rpm max: largest rpm not above the ceiling", inv);
    check("speed command: ceiling clamps after the table",
          motorSpeedCommand(8257, 1000) == 1000 && motorSpeedCommand(4929, 1600) == 800 && motorSpeedCommand(0, 1000) == 0);
    check("mode read-back: 0 throttle, anything else speed",
          motorModeFromRead(0) == 0 && motorModeFromRead(1) == 1 && motorModeFromRead(7) == 1);
    check("setpoint gate, speed mode: 0 / 47 / 2048 / 8257 writable, 8258 not",
          motorSetpointWritable(0, true) && motorSetpointWritable(47, true) && motorSetpointWritable(2048, true)
          && motorSetpointWritable(8257, true) && !motorSetpointWritable(8258, true));
    check("setpoint gate, throttle mode: DShot rule unchanged (47 / 2048 refused, 0 / 48 / 2047 pass)",
          !motorSetpointWritable(47, false) && !motorSetpointWritable(2048, false)
          && motorSetpointWritable(0, false) && motorSetpointWritable(48, false) && motorSetpointWritable(2047, false));

    check("bar 0 -> 0", motorBarFraction(0) == 0.0);
    check("bar 2047 -> 1", motorBarFraction(2047) == 1.0);
    check("bar 4000 clamps to 1", motorBarFraction(4000) == 1.0);
    check("bar negative -> 0", motorBarFraction(-5) == 0.0);
    check("bar NaN -> 0", motorBarFraction(qQNaN()) == 0.0);
    check("bar 1023.5 ~ 0.5", qAbs(motorBarFraction(1023.5) - 0.5) < 1e-9);

    check("state names", motorStateName(0) == "DISARMED" && motorStateName(1) == "INIT"
                         && motorStateName(2) == "ARMED" && motorStateName(3) == "FAILSAFE"
                         && motorStateName(9) == "--");
    return all;
}

#endif // MOTORMATH_H
