#include "attitudeview.h"
#include "attitudeglwidget.h"

#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QFont>
#include <QDateTime>
#include <QDebug>
#include <QtMath>
#include <cmath>

namespace {

int indexOfMeas(const QVector<A2lMeas> &meas, const char *name)
{
    for (int i = 0; i < meas.size(); ++i)
        if (meas[i].name == QLatin1String(name))
            return i;
    return -1;
}

// Result of comparing the quaternion's own roll/pitch against the DAQ's
// AttRoll/Pitch. "checked" is false only in the pitch-clamp/gimbal-lock zone,
// where roll shares a degree of freedom with yaw and is legitimately
// ill-conditioned -- comparing there would be a false positive, not evidence
// of a bug.
struct EulerCrossCheck { bool checked; bool ok; double worstDeltaDeg; };

// Cross-check (SWE1-GUI-001): recompute roll/pitch from the quaternion using
// the exact formulas AurixTricore/src/bsw/Ahrs.c uses to publish
// AttRoll/AttPitch (lines 818-820: same atan2/asin expressions, verified
// against the firmware source, not re-derived from a generic reference) and
// compare against the DAQ-reported values. Unconditional -- runs in every
// build, every sample, not only in debug: a firmware/GUI convention mismatch
// (quaternion order, a sign flip, a swapped axis) must be visible on the
// bench, not only in a log nobody sees on a console-less WIN32_EXECUTABLE
// build. Never the value displayed -- the readout is always the plotted
// channel, verbatim.
//
// Yaw is deliberately EXCLUDED: Ahrs.c lines 822-828 add
// `g_xcpNvm.magDeclDeg` (the A2L's `NvmMagDeclination`, "turns magnetic north
// into true north", default +3.9 deg for Munich) to the quaternion-derived
// yaw before publishing AttYaw. That declination is a CHARACTERISTIC (read
// via SHORT_UPLOAD on the Calibration tab), not a MEASUREMENT on the DAQ list
// this view decodes, so there is no cheap way to recover it here -- comparing
// against it unadjusted flagged every single sample as a "mismatch" purely
// from the declination offset, which is not a bug (caught during review of
// this very check, see the SWE1-GUI-001 status note).
EulerCrossCheck crossCheckEuler(float w, float x, float y, float z,
                                double rollDeg, double pitchDeg)
{
    const double roll = std::atan2(2.0 * (double(w) * x + double(y) * z),
                                   1.0 - 2.0 * (double(x) * x + double(y) * y)) * 180.0 / M_PI;
    const double pitchArg = qBound(-1.0, 2.0 * (double(w) * y - double(z) * x), 1.0);
    const double pitch = std::asin(pitchArg) * 180.0 / M_PI;

    // Ill-conditioned within a few degrees of the +/-90 pitch clamp (Ahrs.h:
    // "pitch ... clamped to +/-90 deg") -- roll trades off against yaw there
    // (gimbal lock), so a large apparent delta is expected and not a
    // mismatch. 85 deg leaves a healthy margin below the clamp while still
    // covering the vast majority of in-flight attitudes.
    if (std::abs(pitch) > 85.0 || std::abs(pitchDeg) > 85.0)
        return { false, true, 0.0 };

    // Roll (like AttRoll's own A2L range) wraps at +/-180 deg. Near that
    // branch cut (board turned over) atan2's float64 result here and the
    // firmware's float32 atan2f can legitimately land a hair on opposite
    // sides of +180/-180 -- an unwrapped subtraction would then read as a
    // ~360 deg "mismatch" for what is actually a sub-degree difference.
    // Compare the short way round, the same fix the removed debug version
    // already applied to yaw.
    double dRoll = std::abs(roll - rollDeg);
    if (dRoll > 180.0)
        dRoll = 360.0 - dRoll;
    const double dPitch = std::abs(pitch - pitchDeg);

    // Tolerance: both sides evaluate the SAME float32 quaternion with the
    // same formula (just once in float32 on the firmware, once in float64
    // here) -- the only disagreement is rounding, on the order of 1e-3 deg
    // for angles in this range, not the 0.01-0.05 deg of two independently
    // measured quantities. 2 deg is still ~2000x that, which is deliberately
    // generous: it comfortably absorbs any residual float32 promotion noise
    // and any relative timing/interpolation nudge in this comparison, while
    // a genuine convention bug (a sign flip or an axis swap) produces tens of
    // degrees, not fractions of one, so it still gets caught.
    constexpr double kToleranceDeg = 2.0;
    const double worst = std::max(dRoll, dPitch);
    return { true, worst <= kToleranceDeg, worst };
}

} // namespace

AttitudeView::AttitudeView(QWidget *parent)
    : QWidget(parent)
{
    m_gl = new AttitudeGLWidget;

    m_selfCheckLbl = new QLabel;
    m_selfCheckLbl->setWordWrap(true);
    m_selfCheckLbl->setStyleSheet("QLabel { color: white; background: #c0392b; font-weight: bold; padding: 4px; }");
    m_selfCheckLbl->setVisible(false);

    m_bindStatusLbl = new QLabel;
    m_bindStatusLbl->setWordWrap(true);
    m_bindStatusLbl->setStyleSheet("QLabel { color: #c0392b; font-weight: bold; }");
    m_bindStatusLbl->setVisible(false);

    QFont mono("Consolas");
    mono.setStyleHint(QFont::Monospace);

    m_rollLbl       = new QLabel("-"); m_rollLbl->setFont(mono);
    m_pitchLbl      = new QLabel("-"); m_pitchLbl->setFont(mono);
    m_yawLbl        = new QLabel("-"); m_yawLbl->setFont(mono);
    m_stateLbl      = new QLabel("Not connected");
    m_crossCheckLbl = new QLabel("-"); m_crossCheckLbl->setFont(mono);
    m_crossCheckLbl->setToolTip(
        "Roll/pitch recomputed from AttQuat0..3 and compared against "
        "AttRoll/AttPitch (SWE1-GUI-001). Yaw is excluded: the firmware adds "
        "the magnetic declination calibration value to it, which is not on "
        "the DAQ list this view decodes.");

    auto *readoutBox = new QGroupBox("Readout (= Plot && Log)");
    auto *form = new QFormLayout(readoutBox);
    form->addRow("Roll:",  m_rollLbl);
    form->addRow("Pitch:", m_pitchLbl);
    form->addRow("Yaw:",   m_yawLbl);
    form->addRow("State:", m_stateLbl);
    form->addRow("Cross-check:", m_crossCheckLbl);

    m_resetViewBtn = new QPushButton("Reset view");
    connect(m_resetViewBtn, &QPushButton::clicked, m_gl, &AttitudeGLWidget::resetView);

    auto *side = new QVBoxLayout;
    side->addWidget(readoutBox);
    side->addWidget(m_resetViewBtn);
    side->addStretch(1);

    auto *sideWidget = new QWidget;
    sideWidget->setLayout(side);
    sideWidget->setMaximumWidth(220);

    auto *row = new QHBoxLayout;
    row->addWidget(m_gl, 1);
    row->addWidget(sideWidget);

    auto *outer = new QVBoxLayout(this);
    outer->addWidget(m_selfCheckLbl);
    outer->addWidget(m_bindStatusLbl);
    outer->addWidget(new QLabel(
        "Drag to orbit, wheel to zoom. The model follows AttQuat0..3 "
        "(body-to-NED); the readout is the plotted AttRoll/AttPitch/AttYaw, "
        "never a value recomputed here."));
    outer->addLayout(row, 1);

    // SWE1-GUI-001: the handedness self-check runs unconditionally at
    // construction (see AttitudeGLWidget). A failure is surfaced here, not
    // only logged -- and the view refuses to go live (see feedSample()),
    // since a wrong handedness would otherwise show a confidently wrong pose.
    if (!m_gl->selfCheckPassed()) {
        m_selfCheckLbl->setText(
            "ATTITUDE SELF-CHECK FAILED - the handedness math did not verify "
            "against its three canonical test cases at startup. The 3D view "
            "is held inert; do not trust the pose. See the debug log for the "
            "per-case detail, or run the attitude_selfcheck test target.");
        m_selfCheckLbl->setVisible(true);
    }

    updateBoundState();
    resetCrossCheck();
}

void AttitudeView::setAvailable(const QVector<A2lMeas> &meas)
{
    m_meas = meas;
    m_idxQuat[0] = indexOfMeas(meas, "AttQuat0");
    m_idxQuat[1] = indexOfMeas(meas, "AttQuat1");
    m_idxQuat[2] = indexOfMeas(meas, "AttQuat2");
    m_idxQuat[3] = indexOfMeas(meas, "AttQuat3");
    m_idxRoll  = indexOfMeas(meas, "AttRoll");
    m_idxPitch = indexOfMeas(meas, "AttPitch");
    m_idxYaw   = indexOfMeas(meas, "AttYaw");
    m_idxState = indexOfMeas(meas, "AttState");

    m_bound = m_idxQuat[0] >= 0 && m_idxQuat[1] >= 0 && m_idxQuat[2] >= 0 && m_idxQuat[3] >= 0;
    updateBoundState();
    // A (possibly different) A2L may resolve AttRoll/AttPitch to different
    // channels entirely -- old counts from the previous file must not carry
    // over and masquerade as evidence about the new one.
    resetCrossCheck();
}

void AttitudeView::updateBoundState()
{
    m_bindStatusLbl->setVisible(!m_bound);
    if (!m_bound) {
        m_bindStatusLbl->setText(
            "AttQuat0..3 not found in the loaded A2L - the attitude view "
            "stays inert until a matching A2L is loaded.");
        showFrozenReadout();
    }
    if (m_gl)
        m_gl->setEnabled(m_bound);
}

void AttitudeView::showFrozenReadout()
{
    m_rollLbl->setText("-");
    m_pitchLbl->setText("-");
    m_yawLbl->setText("-");
    m_stateLbl->setText(m_bound ? "-" : "n/a");
}

void AttitudeView::setConnected(bool connected)
{
    // false -> true: a fresh session on (potentially) a different board or
    // after a reflash -- a mismatch counted against the previous session
    // must not silently keep showing as evidence about this one.
    if (connected && !m_connected)
        resetCrossCheck();

    m_connected = connected;
    if (!connected) {
        if (m_gl)
            m_gl->setLive(false);
        showFrozenReadout();
        m_stateLbl->setText("Disconnected - last pose frozen");
    }
}

void AttitudeView::feedSample(const XcpClient::Measurements &m)
{
    if (!m_bound || !m_connected)
        return;

    double qv[4] = { 0.0, 0.0, 0.0, 0.0 };
    bool ok = true;
    for (int i = 0; i < 4; ++i)
        ok = A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[m_idxQuat[i]], &qv[i]) && ok;
    if (!ok)
        return;   // block not covered yet (e.g. very first sample); keep last pose

    const float w = float(qv[0]), x = float(qv[1]), y = float(qv[2]), z = float(qv[3]);
    if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        return;   // NaN-guard: never feed a broken frame into the renderer

    const bool haveState = m_idxState >= 0;
    int state = -1;
    if (haveState) {
        double sv = 0.0;
        if (A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[m_idxState], &sv) && std::isfinite(sv))
            state = int(sv);
    }
    // Without an AttState channel in the loaded A2L there is no way to tell
    // "running" from "calibrating" -- treat the pose as live-with-caveat
    // (shown as such below) rather than freezing forever with no message,
    // which would look identical to "nothing is happening" (blocker 4).
    // A failed handedness self-check always overrides this: the view must
    // never present a pose it cannot vouch for as live.
    const bool selfCheckOk = m_gl && m_gl->selfCheckPassed();
    const bool live = selfCheckOk && (haveState ? state == 2 : true);
    if (m_gl) {
        m_gl->setLive(live);
        if (live)
            m_gl->pushQuaternion(w, x, y, z);
    }

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    const bool haveRoll  = m_idxRoll  >= 0 && A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[m_idxRoll],  &roll)  && std::isfinite(roll);
    const bool havePitch = m_idxPitch >= 0 && A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[m_idxPitch], &pitch) && std::isfinite(pitch);
    const bool haveYaw   = m_idxYaw   >= 0 && A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[m_idxYaw],   &yaw)   && std::isfinite(yaw);

    m_rollLbl->setText(haveRoll   ? QString::number(roll,  'f', 1) + " deg" : "-");
    m_pitchLbl->setText(havePitch ? QString::number(pitch, 'f', 1) + " deg" : "-");
    m_yawLbl->setText(haveYaw     ? QString::number(yaw,   'f', 1) + " deg" : "-");

    if (!selfCheckOk) {
        m_stateLbl->setText("Self-check FAILED - view held inert");
    } else if (!haveState) {
        m_stateLbl->setText("Running (state unknown - AttState not in A2L)");
    } else {
        switch (state) {
        case 0:  m_stateLbl->setText("Calibrating gyro bias - hold the board still"); break;
        case 1:  m_stateLbl->setText("Aligning - waiting for a usable accel vector"); break;
        case 2:  m_stateLbl->setText("Running"); break;
        case 3:  m_stateLbl->setText("No sensor - IMU absent"); break;
        default: m_stateLbl->setText("-"); break;
        }
    }

    if (!haveRoll || !havePitch) {
        m_crossCheckCurrentChecked   = false;
        m_crossCheckNotCheckedReason = QStringLiteral("angles not in A2L (need AttRoll/AttPitch)");
    } else {
        // Yaw is intentionally not part of this check -- see the comment on
        // crossCheckEuler() (magnetic declination offset, not available on
        // the DAQ list this view decodes).
        const EulerCrossCheck cc = crossCheckEuler(w, x, y, z, roll, pitch);
        if (!cc.checked) {
            m_crossCheckCurrentChecked   = false;
            m_crossCheckNotCheckedReason = QStringLiteral("pitch near +/-90 deg (gimbal-lock zone)");
        } else {
            m_crossCheckCurrentChecked = true;
            ++m_crossCheckSamples;
            if (!cc.ok) {
                ++m_crossCheckMismatches;
                m_lastCrossCheckDeltaDeg = cc.worstDeltaDeg;

                // Rate-limited: the visible counter/label above updates every
                // sample regardless, this only throttles console spam.
                static qint64 lastLogMs = 0;
                const qint64 now = QDateTime::currentMSecsSinceEpoch();
                if (now - lastLogMs >= 1000) {
                    lastLogMs = now;
                    qDebug() << "[Attitude] quaternion/Euler cross-check mismatch, worst delta"
                             << cc.worstDeltaDeg << "deg (DAQ roll/pitch"
                             << roll << pitch << ")";
                }
            }
        }
    }
    updateCrossCheckLabel();
}

void AttitudeView::resetCrossCheck()
{
    m_crossCheckSamples          = 0;
    m_crossCheckMismatches       = 0;
    m_lastCrossCheckDeltaDeg     = 0.0;
    m_crossCheckCurrentChecked   = false;
    m_crossCheckNotCheckedReason = QStringLiteral("no sample yet");
    updateCrossCheckLabel();
}

void AttitudeView::updateCrossCheckLabel()
{
    // A detected mismatch is a real fault and latches (stays shown, keeps
    // counting) regardless of later samples that simply couldn't be
    // checked -- see the header comment for why this differs from the
    // "not checked" state, which is deliberately not sticky.
    if (m_crossCheckMismatches > 0) {
        m_crossCheckLbl->setText(QString("%1 mismatch(es), last %2 deg")
            .arg(m_crossCheckMismatches)
            .arg(m_lastCrossCheckDeltaDeg, 0, 'f', 1));
        m_crossCheckLbl->setStyleSheet("QLabel { color: #c0392b; font-weight: bold; }");
    } else if (m_crossCheckCurrentChecked) {
        m_crossCheckLbl->setText(QString("OK (%1 samples)").arg(m_crossCheckSamples));
        m_crossCheckLbl->setStyleSheet(QString());
    } else {
        // Never shown as "OK": nothing has been compared, so there is
        // nothing yet to be confident about.
        m_crossCheckLbl->setText("not checked: " + m_crossCheckNotCheckedReason);
        m_crossCheckLbl->setStyleSheet("QLabel { color: palette(mid); font-style: italic; }");
    }
}
