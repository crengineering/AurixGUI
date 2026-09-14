#ifndef POSITIONMATH_H
#define POSITIONMATH_H

#include <QtGlobal>
#include <QString>
#include <QPointF>

// Pure logic behind the "Position" panel (SYS1-016 -> SYS2-GUI-003 ->
// SWE1-GUI-006/-007): window scaling, the NED sign flip, the anchoring truth
// table, trail age-fade and the live/stale gate. No QWidget, no A2L, no XCP
// dependency here on purpose -- so this file (and only this file) is what
// position_selfcheck links, the same isolation attitudeglwidget.h/.cpp gives
// attitude_selfcheck for the handedness math.

// Design default (SYS2-GUI-003, stated by aurix-gui in its report): the
// trail keeps the last 60 s, not a user setting. ONE definition, shared by
// PositionView::pruneTrail's window and PositionTrailWidget's age-fade
// divisor (posTrailAlpha) -- review finding MINOR 7 was the two disagreeing.
inline constexpr qint64 kPosTrailLengthMs = 60000;

// The three selectable trail/altitude windows (SYS2-GUI-003 pt.1/2): full
// width/height of the plot each represents, in metres.
enum class PosWindow { W2m, W5m, W20m };

double posWindowMetres(PosWindow w);

// Converts a north/east offset from the view centre (metres) into a
// normalized position in [-1, 1] per axis, CLAMPED at the window's edge --
// x = east/half (right = +1), y = -north/half (up = -1, screen convention).
// Used for points that are meant to be pinned to the frame (the origin
// marker, the current-position marker, which sits at offset (0,0) by
// definition). NOT for the trail polyline -- see posNormalizeUnclamped().
QPointF posNormalize(double northOffsetM, double eastOffsetM, PosWindow w);

// Same mapping, WITHOUT the clamp -- a point outside the window maps outside
// [-1,1]. The trail widget draws these coordinates under a QPainter clip
// rect instead of snapping them to the border: SYS2-GUI-003 says the panel
// "must not snap or hide the wander", and clamping every off-window trail
// point onto the border would paint a path along the frame that was never
// flown (review finding MAJOR 4).
QPointF posNormalizeUnclamped(double northOffsetM, double eastOffsetM, PosWindow w);

// NED sign flip, applied exactly once, at display (SYS2-GUI-003 pt.2):
// Up = -Down for any quantity carried on the down axis. Used for BOTH
// NavPosDown -> Up (metres) and NavVelDown -> vertical velocity (m/s) --
// one flip site for both, so the two can never disagree (review finding
// MAJOR 2: the velocity row used to negate inline, separately).
inline double posUpFromDown(double downValue) { return -downValue; }

// Trail-point opacity by age (newest brightest, SYS2-GUI-003 pt.1): 1.0 at
// age 0, floor 0.08 at/after trailLengthMs so an old point never disappears
// completely into the same alpha as "nothing there". Shared by
// PositionTrailWidget::paintEvent() and this self-check.
double posTrailAlpha(qint64 ageMs, qint64 trailLengthMs = kPosTrailLengthMs);

// The live/stale gate PositionView applies to the trail and altitude bar
// (SWE1-GUI-007): without an AttState channel in the A2L, live-with-caveat
// (never freeze forever silently, same rule AttitudeView applies); with one,
// only AttState == 2 (running) counts as live.
bool posIsLive(bool haveAttState, int attState);

// Three-state anchoring text (SYS2-GUI-003 pt.4 -- "Readouts", renumbered
// when pt.3 "Speed at the drone marker" was inserted 2026-09-14), never
// confusable with one
// another:
//   horizontal: "GNSS anchored (<fix>, n sats, hAcc x m)" iff navHorizontalOk
//               AND gnssNavOk, else "dead reckoning - drifts (<fix detail>)"
//               -- <fix detail> names GnssFixType (no fix / 2D / 3D /
//               GNSS+DR), the channel that separates "no fix" from "fix
//               exists but not yet trusted" (review finding MAJOR 1).
//   vertical:   "baro anchored" iff navVerticalOk, else "no vertical anchor"
// gnssFixType < 0 means "not available" (channel missing from the A2L);
// gnssNumSats < 0 / gnssHAccuracyM < 0 likewise -- all render as "n/a"
// rather than a bogus number.
struct PosAnchorState {
    QString horizontalText;
    bool    horizontalAnchored = false;
    QString verticalText;
    bool    verticalAnchored   = false;
};

PosAnchorState posAnchorState(bool navHorizontalOk, bool gnssNavOk, bool navVerticalOk,
                               int gnssFixType, int gnssNumSats, double gnssHAccuracyM);

// "Speed at the drone marker" (SYS2-GUI-003 pt.3, added 2026-09-14 at
// Chris's request, acceptance clause (g)): ground speed and the
// velocity-arrow direction drawn at the trail's current-position marker.
// Below this ground speed (the NEO-M9N's own velocity accuracy, per its
// datasheet/AurixTricore/docs/GNSS_UBX.md) the arrow is not meaningful --
// draw the label only, no arrow.
inline constexpr double kPosArrowMinSpeedMs = 0.05;

// sqrt(velNorth^2 + velEast^2), m/s.
double posGroundSpeed(double velNorthMs, double velEastMs);

// Screen-space (east=+x, north=-y, i.e. north up/east right map convention)
// unit direction of travel, from the course over ground
// atan2(velEast, velNorth) -- or (0,0) if the ground speed is below
// kPosArrowMinSpeedMs, meaning "do not draw an arrow" (the label still
// shows the speed regardless).
QPointF posVelocityArrowDir(double velNorthMs, double velEastMs);

// Exercises every function above against known cases -- headless, no
// GL/display/QApplication needed, run by the position_selfcheck CMake target
// (build.bat / CI), mirroring attitudeSelfCheck(). *report, if given, gets a
// human-readable pass/fail table. Returns true iff every case passed.
bool positionSelfCheck(QString *report = nullptr);

#endif // POSITIONMATH_H
