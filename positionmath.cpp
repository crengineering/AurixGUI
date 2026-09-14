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

QPointF posNormalize(double northOffsetM, double eastOffsetM, PosWindow w)
{
    const double half = posWindowMetres(w) / 2.0;
    if (half <= 0.0)
        return QPointF(0.0, 0.0);
    const double x = qBound(-1.0, eastOffsetM / half, 1.0);
    const double y = qBound(-1.0, -northOffsetM / half, 1.0);
    return QPointF(x, y);
}

PosAnchorState posAnchorState(bool navHorizontalOk, bool gnssNavOk, bool navVerticalOk,
                               int gnssNumSats, double gnssHAccuracyM)
{
    PosAnchorState s;

    // "GNSS anchored" only when BOTH flags say so (SYS2-GUI-003 pt.3) -- one
    // without the other is still dead reckoning, e.g. a fix arriving before
    // the fusion filter has actually latched onto it.
    s.horizontalAnchored = navHorizontalOk && gnssNavOk;
    if (s.horizontalAnchored) {
        const QString sats = gnssNumSats >= 0 ? QString::number(gnssNumSats) : QStringLiteral("n/a");
        const QString hAcc = gnssHAccuracyM >= 0.0
                                  ? QString::number(gnssHAccuracyM, 'f', 1)
                                  : QStringLiteral("n/a");
        s.horizontalText = QStringLiteral("GNSS anchored (%1 sats, hAcc %2 m)").arg(sats, hAcc);
    } else {
        s.horizontalText = QStringLiteral("dead reckoning - drifts");
    }

    s.verticalAnchored = navVerticalOk;
    s.verticalText = s.verticalAnchored ? QStringLiteral("baro anchored")
                                         : QStringLiteral("no vertical anchor");
    return s;
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

    // Normalize: centre stays centre in every window.
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
    // South offset beyond the window clamps to the bottom edge, not off-scale.
    {
        const QPointF p = posNormalize(-100.0, 0.0, PosWindow::W2m);
        allOk &= checkCase(report, nearlyEq(p.y(), 1.0), "normalize: far south clamps to y=+1");
    }

    // Sign flip.
    allOk &= checkCase(report, nearlyEq(posUpFromDown(0.5), -0.5), "up = -down (0.5 -> -0.5)");
    allOk &= checkCase(report, nearlyEq(posUpFromDown(-1.0), 1.0), "up = -down (-1.0 -> 1.0)");
    allOk &= checkCase(report, nearlyEq(posUpFromDown(0.0), 0.0), "up = -down (0.0 -> 0.0)");

    // Anchoring truth table (SYS2-GUI-003 pt.3) -- every combination that
    // could be confused with another must not be.
    {
        const PosAnchorState s = posAnchorState(false, false, false, -1, -1.0);
        allOk &= checkCase(report, !s.horizontalAnchored && !s.verticalAnchored
                                        && s.horizontalText.contains("dead reckoning")
                                        && s.verticalText.contains("no vertical anchor"),
                            "anchor: nothing ok -> dead reckoning + no vertical anchor");
    }
    {
        // GnssNavOk true but NavHorizontalOk false: NOT anchored -- a fix
        // exists but fusion has not latched it yet.
        const PosAnchorState s = posAnchorState(false, true, false, 9, 3.0);
        allOk &= checkCase(report, !s.horizontalAnchored && s.horizontalText.contains("dead reckoning"),
                            "anchor: GnssNavOk alone is NOT enough (needs NavHorizontalOk too)");
    }
    {
        // NavHorizontalOk true but GnssNavOk false: NOT anchored -- the
        // filter's own flag against the current fix's own gate disagreeing.
        const PosAnchorState s = posAnchorState(true, false, false, 9, 3.0);
        allOk &= checkCase(report, !s.horizontalAnchored,
                            "anchor: NavHorizontalOk alone is NOT enough (needs GnssNavOk too)");
    }
    {
        const PosAnchorState s = posAnchorState(true, true, true, 11, 1.5);
        allOk &= checkCase(report, s.horizontalAnchored && s.verticalAnchored
                                        && s.horizontalText.contains("GNSS anchored")
                                        && s.horizontalText.contains("11")
                                        && s.verticalText.contains("baro anchored"),
                            "anchor: both ok -> GNSS anchored (11 sats) + baro anchored");
    }
    {
        // Missing GNSS detail channels (sats/hAcc not in the A2L) render as
        // n/a instead of a bogus 0 or -1.
        const PosAnchorState s = posAnchorState(true, true, false, -1, -1.0);
        allOk &= checkCase(report, s.horizontalAnchored && s.horizontalText.contains("n/a"),
                            "anchor: anchored with missing sats/hAcc shows n/a, not a bogus number");
    }

    return allOk;
}
