#ifndef POSITIONMATH_H
#define POSITIONMATH_H

#include <QString>
#include <QPointF>

// Pure logic behind the "Position" panel (SYS1-016 -> SYS2-GUI-003 ->
// SWE1-GUI-006/-007): window scaling, the NED sign flip and the anchoring
// truth table. No QWidget, no A2L, no XCP dependency here on purpose -- so
// this file (and only this file) is what position_selfcheck links, the same
// isolation attitudeglwidget.h/.cpp gives attitude_selfcheck for the
// handedness math.

// The three selectable trail/altitude windows (SYS2-GUI-003 pt.1/2): full
// width/height of the plot each represents, in metres.
enum class PosWindow { W2m, W5m, W20m };

double posWindowMetres(PosWindow w);

// Converts a north/east offset from the view centre (metres) into a
// normalized position in [-1, 1] per axis, clamped at the window's edge --
// x = east/half (right = +1), y = -north/half (up = -1, screen convention).
// Shared, tested logic for PositionTrailWidget::paintEvent() and this
// self-check, so the two can never silently disagree.
QPointF posNormalize(double northOffsetM, double eastOffsetM, PosWindow w);

// NED sign flip, applied exactly once, at display (SYS2-GUI-003 pt.2):
// Up = -NavPosDown ("down" positive per AurixTricore/src/bsw/fusion.h).
inline double posUpFromDown(double navPosDownM) { return -navPosDownM; }

// Three-state anchoring text (SYS2-GUI-003 pt.3), never confusable with one
// another:
//   horizontal: "GNSS anchored (n sats, hAcc x m)" iff navHorizontalOk AND
//               gnssNavOk, else "dead reckoning - drifts"
//   vertical:   "baro anchored" iff navVerticalOk, else "no vertical anchor"
// gnssNumSats < 0 / gnssHAccuracyM < 0 mean "not available" (channel missing
// from the A2L) and are rendered as "n/a" rather than a bogus number.
struct PosAnchorState {
    QString horizontalText;
    bool    horizontalAnchored = false;
    QString verticalText;
    bool    verticalAnchored   = false;
};

PosAnchorState posAnchorState(bool navHorizontalOk, bool gnssNavOk, bool navVerticalOk,
                               int gnssNumSats, double gnssHAccuracyM);

// Exercises posWindowMetres()/posNormalize()/posUpFromDown()/posAnchorState()
// against known cases -- headless, no GL/display/QApplication needed, run by
// the position_selfcheck CMake target (build.bat / CI), mirroring
// attitudeSelfCheck(). *report, if given, gets a human-readable pass/fail
// table. Returns true iff every case passed.
bool positionSelfCheck(QString *report = nullptr);

#endif // POSITIONMATH_H
