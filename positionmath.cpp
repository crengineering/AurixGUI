#include "positionmath.h"

#include <QtGlobal>
#include <cmath>

double posWindowMetres(PosWindow w)
{
    switch (w) {
    case PosWindow::W2m:  return 2.0;
    case PosWindow::W5m:  return 5.0;
    case PosWindow::W20m: return 20.0;
    }
    return 5.0;
}

namespace {

QPointF normalizeImpl(double northOffsetM, double eastOffsetM, PosWindow w, bool clamp)
{
    const double half = posWindowMetres(w) / 2.0;
    if (half <= 0.0)
        return QPointF(0.0, 0.0);
    double x = eastOffsetM / half;
    double y = -northOffsetM / half;
    if (clamp) {
        x = qBound(-1.0, x, 1.0);
        y = qBound(-1.0, y, 1.0);
    }
    return QPointF(x, y);
}

// GnssFixType (UBX-NAV-PVT): 0 no fix, 2 2D, 3 3D, 4 GNSS+DR -- see
// AurixTricore/docs/AurixTricore.a2l's MEASUREMENT comment. Any other value
// (channel present but out of the documented range) reports itself rather
// than guessing.
QString fixTypeText(int fixType)
{
    switch (fixType) {
    case 0:  return QStringLiteral("no fix");
    case 2:  return QStringLiteral("2D fix");
    case 3:  return QStringLiteral("3D fix");
    case 4:  return QStringLiteral("GNSS+DR fix");
    default: return QStringLiteral("fix type %1").arg(fixType);
    }
}

} // namespace

QPointF posNormalize(double northOffsetM, double eastOffsetM, PosWindow w)
{
    return normalizeImpl(northOffsetM, eastOffsetM, w, /*clamp=*/true);
}

QPointF posNormalizeUnclamped(double northOffsetM, double eastOffsetM, PosWindow w)
{
    return normalizeImpl(northOffsetM, eastOffsetM, w, /*clamp=*/false);
}

double posTrailAlpha(qint64 ageMs, qint64 trailLengthMs)
{
    if (trailLengthMs <= 0)
        return 1.0;
    const double ageFrac = qBound(0.0, double(ageMs) / double(trailLengthMs), 1.0);
    return qBound(0.08, 1.0 - ageFrac, 1.0);
}

bool posIsLive(bool haveAttState, int attState)
{
    return haveAttState ? (attState == 2) : true;
}

PosAnchorState posAnchorState(bool navHorizontalOk, bool gnssNavOk, bool navVerticalOk,
                               int gnssFixType, int gnssNumSats, double gnssHAccuracyM,
                               bool haveGnssTrusted, bool gnssTrusted)
{
    PosAnchorState s;

    const QString sats = gnssNumSats >= 0 ? QString::number(gnssNumSats) : QStringLiteral("n/a");
    const QString hAcc = gnssHAccuracyM >= 0.0
                              ? QString::number(gnssHAccuracyM, 'f', 1) + " m"
                              : QStringLiteral("n/a");
    const QString fix = gnssFixType >= 0 ? fixTypeText(gnssFixType) : QStringLiteral("fix n/a");

    if (haveGnssTrusted) {
        // Three-way read (SWE1-FW-011, fw >= 1.19.26): navHorizontalOk
        // already implies gnssTrusted AND fresh (fusion.c), so "anchored"
        // needs nothing further; the trust flag exists to say WHY it is
        // *not* anchored, which the two-way reading below cannot.
        s.horizontalAnchored = navHorizontalOk;
        if (s.horizontalAnchored) {
            s.horizontalText = QStringLiteral("GNSS anchored (%1, %2 sats, hAcc %3)").arg(fix, sats, hAcc);
        } else if (!gnssNavOk) {
            // No fix at all -- the firmware HOLDS (never dead-reckons):
            // North/East simply do not move.
            s.horizontalText = QStringLiteral("no fix - position held (%1, %2 sats)").arg(fix, sats);
        } else if (!gnssTrusted) {
            // A fix exists but its own reported hAcc fails the trust gate
            // (FusGnssHAccMax, kPosGnssHAccMaxM) -- frozen, not drifting:
            // North/East are held exactly where they were at the last
            // trusted fix.
            s.horizontalText = QStringLiteral("frozen - GNSS untrusted (hAcc %1 > %2 m)")
                                    .arg(hAcc, QString::number(kPosGnssHAccMaxM, 'f', 1));
        } else {
            // Trusted and fixed but not yet fresh/latched (e.g. the
            // tangent-plane origin has not been set yet) -- a transient
            // state, distinct from both of the above.
            s.horizontalText = QStringLiteral("not yet anchored (%1, hAcc %2)").arg(fix, hAcc);
        }
    } else {
        // Legacy path (fw < 1.19.26 / A2L without NavGnssTrusted): only the
        // two-way reading is possible -- "no fix" and "fix not yet trusted"
        // look the same from here, which the "[legacy: ...]" tag says
        // outright instead of silently reusing the old, now-inaccurate
        // "dead reckoning" wording (the firmware never actually dead-reckons
        // -- SWE1-GUI-007 amendment, 2026-09-15).
        s.horizontalAnchored = navHorizontalOk && gnssNavOk;
        if (s.horizontalAnchored) {
            s.horizontalText = QStringLiteral("GNSS anchored (%1, %2 sats, hAcc %3)").arg(fix, sats, hAcc);
        } else {
            // GnssFixType is what separates "no fix at all" from "a fix
            // exists but is not (yet) trusted for the horizontal channels"
            // (review finding MAJOR 1). No fix at all: sats is the
            // informative number (hAcc is meaningless without one). A fix
            // of any quality: hAcc is the informative number.
            const QString detail = (gnssFixType == 0)
                                        ? QStringLiteral("%1, %2 sats").arg(fix, sats)
                                        : QStringLiteral("%1, hAcc %2").arg(fix, hAcc);
            s.horizontalText = QStringLiteral("not anchored - position held [legacy: %1]").arg(detail);
        }
    }

    s.verticalAnchored = navVerticalOk;
    s.verticalText = s.verticalAnchored ? QStringLiteral("baro anchored")
                                         : QStringLiteral("no vertical anchor");
    return s;
}

double posGroundSpeed(double velNorthMs, double velEastMs)
{
    return std::sqrt(velNorthMs * velNorthMs + velEastMs * velEastMs);
}

QPointF posVelocityArrowDir(double velNorthMs, double velEastMs)
{
    const double speed = posGroundSpeed(velNorthMs, velEastMs);
    if (speed < kPosArrowMinSpeedMs)
        return QPointF(0.0, 0.0);   // below the receiver's velocity accuracy: label only

    // Course over ground, bearing from north, clockwise positive (the
    // conventional definition, matching Chris's spec wording literally):
    // atan2(east, north). Converting it back into a screen unit vector
    // (north up = -y, east right = +x) gives sin/-cos of that angle, which
    // is numerically identical to the direct (east, north)/speed vector --
    // both are computed and cross-checked against the four cardinal
    // directions in positionSelfCheck() below.
    const double courseRad = std::atan2(velEastMs, velNorthMs);
    return QPointF(std::sin(courseRad), -std::cos(courseRad));
}

namespace {

bool checkCase(QString *report, bool ok, const QString &name)
{
    if (report)
        *report += (ok ? QStringLiteral("[PASS] ") : QStringLiteral("[FAIL] ")) + name + "\n";
    return ok;
}

bool nearlyEq(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

} // namespace

bool positionSelfCheck(QString *report)
{
    bool allOk = true;

    // Window sizes.
    allOk &= checkCase(report, nearlyEq(posWindowMetres(PosWindow::W2m), 2.0), "window 2 m");
    allOk &= checkCase(report, nearlyEq(posWindowMetres(PosWindow::W5m), 5.0), "window 5 m");
    allOk &= checkCase(report, nearlyEq(posWindowMetres(PosWindow::W20m), 20.0), "window 20 m");

    // Normalize (clamped): centre stays centre in every window.
    {
        const QPointF p = posNormalize(0.0, 0.0, PosWindow::W5m);
        allOk &= checkCase(report, nearlyEq(p.x(), 0.0) && nearlyEq(p.y(), 0.0),
                            "normalize: centre -> (0,0)");
    }
    // North offset = +half-window -> top edge (y = -1), east untouched.
    {
        const QPointF p = posNormalize(1.0, 0.0, PosWindow::W2m); // half = 1 m
        allOk &= checkCase(report, nearlyEq(p.x(), 0.0) && nearlyEq(p.y(), -1.0),
                            "normalize: north = +half -> top edge (y=-1)");
    }
    // East offset = +half-window -> right edge (x = +1).
    {
        const QPointF p = posNormalize(0.0, 2.5, PosWindow::W5m); // half = 2.5 m
        allOk &= checkCase(report, nearlyEq(p.x(), 1.0) && nearlyEq(p.y(), 0.0),
                            "normalize: east = +half -> right edge (x=+1)");
    }
    // South offset beyond the window CLAMPS to the bottom edge (this is the
    // origin/current-position marker mapping, which is meant to pin to frame).
    {
        const QPointF p = posNormalize(-100.0, 0.0, PosWindow::W2m);
        allOk &= checkCase(report, nearlyEq(p.y(), 1.0), "normalize (clamped): far south clamps to y=+1");
    }
    // Unclamped: the SAME far offset must NOT land on the border -- this is
    // what the trail polyline uses, precisely so a point outside the window
    // is clipped, not snapped onto the frame as if it were flown along it
    // (review finding MAJOR 4).
    {
        const QPointF p = posNormalizeUnclamped(-100.0, 0.0, PosWindow::W2m); // half = 1 m
        allOk &= checkCase(report, nearlyEq(p.y(), 100.0),
                            "normalize (unclamped): far south does NOT clamp to the border");
    }
    {
        const QPointF p = posNormalizeUnclamped(0.0, 0.0, PosWindow::W5m);
        allOk &= checkCase(report, nearlyEq(p.x(), 0.0) && nearlyEq(p.y(), 0.0),
                            "normalize (unclamped): centre -> (0,0), same as clamped");
    }

    // Sign flip -- shared by NavPosDown->Up and NavVelDown->vertical velocity.
    allOk &= checkCase(report, nearlyEq(posUpFromDown(0.5), -0.5), "up = -down (0.5 -> -0.5)");
    allOk &= checkCase(report, nearlyEq(posUpFromDown(-1.0), 1.0), "up = -down (-1.0 -> 1.0)");
    allOk &= checkCase(report, nearlyEq(posUpFromDown(0.0), 0.0), "up = -down (0.0 -> 0.0)");

    // Trail age-fade: newest (age 0) is fully opaque, an old point never goes
    // fully transparent (floor 0.08), and it decreases monotonically between.
    allOk &= checkCase(report, nearlyEq(posTrailAlpha(0, 60000), 1.0), "trail alpha: age 0 -> 1.0 (brightest)");
    allOk &= checkCase(report, nearlyEq(posTrailAlpha(60000, 60000), 0.08), "trail alpha: age = length -> floor 0.08");
    allOk &= checkCase(report, nearlyEq(posTrailAlpha(120000, 60000), 0.08), "trail alpha: age > length -> still floor 0.08, not 0");
    allOk &= checkCase(report, posTrailAlpha(15000, 60000) > posTrailAlpha(45000, 60000),
                        "trail alpha: monotonically fades with age");

    // Live/stale gate (SWE1-GUI-007): no AttState channel -> live-with-
    // caveat; with one, only state 2 (running) is live.
    allOk &= checkCase(report, posIsLive(false, -1) == true, "live gate: no AttState channel -> live-with-caveat");
    allOk &= checkCase(report, posIsLive(true, 2) == true, "live gate: AttState running -> live");
    allOk &= checkCase(report, posIsLive(true, 0) == false, "live gate: AttState calibrating -> not live");
    allOk &= checkCase(report, posIsLive(true, 1) == false, "live gate: AttState aligning -> not live");
    allOk &= checkCase(report, posIsLive(true, 3) == false, "live gate: AttState no-sensor -> not live");

    // Ground speed and velocity arrow (SYS2-GUI-003 pt.3, acceptance (g)).
    allOk &= checkCase(report, nearlyEq(posGroundSpeed(3.0, 4.0), 5.0), "ground speed: 3/4/5 triangle");
    allOk &= checkCase(report, nearlyEq(posGroundSpeed(0.0, 0.0), 0.0), "ground speed: at rest -> 0");
    {
        // Just below the 0.05 m/s threshold: no arrow, label-only.
        const QPointF d = posVelocityArrowDir(0.03, 0.03); // speed ~0.0424 m/s
        allOk &= checkCase(report, nearlyEq(d.x(), 0.0) && nearlyEq(d.y(), 0.0),
                            "velocity arrow: below 0.05 m/s -> no arrow (0,0), label only");
    }
    {
        // Cardinal directions: north/east/south/west must map onto the
        // correct SCREEN direction (north up, east right, map convention).
        const QPointF north = posVelocityArrowDir(1.0, 0.0);
        allOk &= checkCase(report, nearlyEq(north.x(), 0.0) && nearlyEq(north.y(), -1.0),
                            "velocity arrow: due north -> screen up (0,-1)");
        const QPointF east = posVelocityArrowDir(0.0, 1.0);
        allOk &= checkCase(report, nearlyEq(east.x(), 1.0) && nearlyEq(east.y(), 0.0),
                            "velocity arrow: due east -> screen right (1,0)");
        const QPointF south = posVelocityArrowDir(-1.0, 0.0);
        allOk &= checkCase(report, nearlyEq(south.x(), 0.0) && nearlyEq(south.y(), 1.0),
                            "velocity arrow: due south -> screen down (0,1)");
        const QPointF west = posVelocityArrowDir(0.0, -1.0);
        allOk &= checkCase(report, nearlyEq(west.x(), -1.0) && nearlyEq(west.y(), 0.0),
                            "velocity arrow: due west -> screen left (-1,0)");
    }
    // Vertical rate uses the SAME flip site as Up position (review finding
    // MAJOR 2 / the coordinator's scope addition): NavVelDown -> Up rate.
    allOk &= checkCase(report, nearlyEq(posUpFromDown(0.3), -0.3), "vertical rate: up = -down (0.3 -> -0.3 m/s)");
    allOk &= checkCase(report, nearlyEq(posUpFromDown(-0.2), 0.2), "vertical rate: up = -down (-0.2 -> 0.2 m/s)");

    // Anchoring truth table, LEGACY path (SYS2-GUI-003 pt.4, haveGnssTrusted
    // = false -- A2L without NavGnssTrusted) -- every combination that could
    // be confused with another must not be, and none of them may say "dead
    // reckoning" any more (SWE1-GUI-007 amendment, 2026-09-15: the firmware
    // never actually dead-reckons).
    {
        const PosAnchorState s = posAnchorState(false, false, false, 0, 0, -1.0, false, false);
        allOk &= checkCase(report, !s.horizontalAnchored && !s.verticalAnchored
                                        && !s.horizontalText.contains("dead reckoning")
                                        && s.horizontalText.contains("[legacy")
                                        && s.horizontalText.contains("no fix")
                                        && s.verticalText.contains("no vertical anchor"),
                            "anchor (legacy): nothing ok, no fix -> not anchored [legacy] + no vertical anchor");
    }
    {
        // GnssNavOk true but NavHorizontalOk false: NOT anchored -- a fix
        // exists but fusion has not latched it yet. GnssFixType = 2 (2D) is
        // named in the detail, not hidden.
        const PosAnchorState s = posAnchorState(false, true, false, 2, 9, 3.0, false, false);
        allOk &= checkCase(report, !s.horizontalAnchored
                                        && !s.horizontalText.contains("dead reckoning")
                                        && s.horizontalText.contains("[legacy")
                                        && s.horizontalText.contains("2D fix"),
                            "anchor (legacy): GnssNavOk alone is NOT enough, fix type still shown (2D fix)");
    }
    {
        // NavHorizontalOk true but GnssNavOk false: NOT anchored -- the
        // filter's own flag against the current fix's own gate disagreeing.
        const PosAnchorState s = posAnchorState(true, false, false, 3, 9, 3.0, false, false);
        allOk &= checkCase(report, !s.horizontalAnchored,
                            "anchor (legacy): NavHorizontalOk alone is NOT enough (needs GnssNavOk too)");
    }
    {
        const PosAnchorState s = posAnchorState(true, true, true, 3, 11, 1.5, false, false);
        allOk &= checkCase(report, s.horizontalAnchored && s.verticalAnchored
                                        && s.horizontalText.contains("GNSS anchored")
                                        && s.horizontalText.contains("3D fix")
                                        && s.horizontalText.contains("11")
                                        && s.verticalText.contains("baro anchored"),
                            "anchor (legacy): both ok -> GNSS anchored (3D fix, 11 sats) + baro anchored");
    }
    {
        // Missing GNSS detail channels (fix type/sats/hAcc not in the A2L)
        // render as n/a instead of a bogus number.
        const PosAnchorState s = posAnchorState(true, true, false, -1, -1, -1.0, false, false);
        allOk &= checkCase(report, s.horizontalAnchored && s.horizontalText.contains("n/a"),
                            "anchor (legacy): anchored with missing fix/sats/hAcc shows n/a, not a bogus number");
    }

    // Anchoring truth table, THREE-WAY path (SWE1-GUI-007 amendment,
    // haveGnssTrusted = true -- NavGnssTrusted present, fw >= 1.19.26): every
    // combination of trusted/navOk/horizontalOk that Chris needs to tell
    // apart on the bench.
    {
        // navHorizontalOk true -> anchored, regardless of the other flags
        // (it already implies trusted+fresh in the firmware).
        const PosAnchorState s = posAnchorState(true, true, true, 3, 11, 1.2, true, true);
        allOk &= checkCase(report, s.horizontalAnchored && s.verticalAnchored
                                        && s.horizontalText.contains("GNSS anchored")
                                        && s.horizontalText.contains("11"),
                            "anchor (3-way): horizontalOk -> GNSS anchored (3D fix, 11 sats)");
    }
    {
        // No fix at all: "no fix - position held", never "dead reckoning".
        const PosAnchorState s = posAnchorState(false, false, false, 0, 0, -1.0, true, false);
        allOk &= checkCase(report, !s.horizontalAnchored
                                        && s.horizontalText.contains("no fix")
                                        && s.horizontalText.contains("position held")
                                        && !s.horizontalText.contains("dead reckoning"),
                            "anchor (3-way): no fix -> no fix - position held");
    }
    {
        // A fix exists (GnssNavOk) but is not trusted (hAcc too high indoors):
        // "frozen - GNSS untrusted", names the threshold, never "dead
        // reckoning".
        const PosAnchorState s = posAnchorState(false, true, false, 3, 9, 6.4, true, false);
        allOk &= checkCase(report, !s.horizontalAnchored
                                        && s.horizontalText.contains("frozen")
                                        && s.horizontalText.contains("GNSS untrusted")
                                        && s.horizontalText.contains("6.4 m")
                                        && s.horizontalText.contains(QString::number(kPosGnssHAccMaxM, 'f', 1))
                                        && !s.horizontalText.contains("dead reckoning"),
                            "anchor (3-way): navOk but untrusted -> frozen - GNSS untrusted (hAcc 6.4 m > 4.0 m)");
    }
    {
        // Trusted and fixed but not yet fresh/latched (e.g. origin not set
        // yet) -- distinct text from both of the above, still not anchored.
        const PosAnchorState s = posAnchorState(false, true, false, 3, 9, 1.0, true, true);
        allOk &= checkCase(report, !s.horizontalAnchored
                                        && s.horizontalText.contains("not yet anchored")
                                        && !s.horizontalText.contains("frozen")
                                        && !s.horizontalText.contains("no fix"),
                            "anchor (3-way): trusted+fixed but not fresh -> not yet anchored (transient)");
    }

    // LOCK badge (SWE1-FW-014 -> SWE1-GUI-007 amendment): absent channel
    // never shows a false LOCK; present + 0 does not either; only present +
    // 1 does.
    allOk &= checkCase(report, posShowLockBadge(false, true) == false,
                        "lock badge: channel absent -> never shown, even if the raw value looks set");
    allOk &= checkCase(report, posShowLockBadge(true, false) == false,
                        "lock badge: channel present, not locked -> not shown");
    allOk &= checkCase(report, posShowLockBadge(true, true) == true,
                        "lock badge: channel present and locked -> shown");

    return allOk;
}
