#ifndef ATTITUDEVIEW_H
#define ATTITUDEVIEW_H

#include <QWidget>
#include <QVector>
#include <QString>
#include "a2lmodel.h"
#include "xcpclient.h"

class QLabel;
class QPushButton;
class AttitudeGLWidget;

// "Attitude" sub-tab (SYS1-015 -> SYS2-GUI-001/002 -> SWE1-GUI-001..004): a
// 3D quadrocopter model that follows the firmware's body-to-NED attitude
// quaternion over the existing XCP DAQ list (event channel 0, 100 ms) -- the
// same delivery path PlotPane uses, not a second XCP session.
//
// Channels are resolved by NAME against the loaded A2L (AttQuat0..3,
// AttRoll/Pitch/Yaw, AttState), mirroring PlotPane::setAvailable(); if the
// four quaternion channels are missing the view says so and stays inert.
class AttitudeView : public QWidget
{
    Q_OBJECT

public:
    explicit AttitudeView(QWidget *parent = nullptr);

    // Re-resolve the channels against a (possibly new) A2L MEASUREMENT list.
    void setAvailable(const QVector<A2lMeas> &meas);

public slots:
    // Mirrors XcpPanel's own connected/disconnected state (the same signal
    // the rest of the GUI uses for stale/disconnect) -- never presents the
    // last pose as live once the link is down.
    void setConnected(bool connected);

    // One decoded DAQ sample, already assembled by XcpClient -- no I/O here,
    // a pure in-memory decode of bytes that already arrived.
    void feedSample(const XcpClient::Measurements &m);

private:
    void updateBoundState();
    void showFrozenReadout();

    AttitudeGLWidget *m_gl               = nullptr;
    QLabel           *m_rollLbl          = nullptr;
    QLabel           *m_pitchLbl         = nullptr;
    QLabel           *m_yawLbl           = nullptr;
    QLabel           *m_stateLbl         = nullptr;
    QLabel           *m_crossCheckLbl    = nullptr;
    QLabel           *m_bindStatusLbl    = nullptr;
    QLabel           *m_selfCheckLbl     = nullptr;   // red banner, self-check FAILED
    QPushButton      *m_resetViewBtn     = nullptr;

    QVector<A2lMeas> m_meas;
    int  m_idxQuat[4] = { -1, -1, -1, -1 };
    int  m_idxRoll    = -1;
    int  m_idxPitch   = -1;
    int  m_idxYaw     = -1;
    int  m_idxState   = -1;
    bool m_bound      = false;   // AttQuat0..3 all resolved against the A2L
    bool m_connected  = false;

    // Quaternion-vs-Euler cross-check (SWE1-GUI-001): runs unconditionally,
    // every sample, not just in debug builds -- a firmware/GUI convention
    // mismatch must be visible on the bench, not only in a log nobody sees
    // on a console-less build.
    //
    // Three distinct, visibly different states (never conflate "not checked"
    // with "OK" -- a sample that was never compared is not evidence of
    // anything):
    //   "not checked: <reason>" -- the CURRENT/latest sample could not be
    //       compared (no sample yet, pitch near +/-90 deg gimbal lock, or
    //       AttRoll/AttPitch missing from the A2L). Reflects the live,
    //       instantaneous state, so it correctly reappears every time Chris
    //       pitches through the gimbal-lock zone even after an earlier OK.
    //   "OK (n samples)" -- at least one sample has been checked, all of
    //       them (n = m_crossCheckSamples) matched within tolerance, and the
    //       current sample was one of them.
    //   "n mismatch(es), last x.x deg" -- sticky once any checked sample
    //       fails: stays shown (and counting) through any later "not
    //       checked" sample, until an explicit reset (reconnect or A2L
    //       reload) -- a detected mismatch is a real fault, worth latching,
    //       unlike the benign "can't check right now" state.
    void updateCrossCheckLabel();
    void resetCrossCheck();     // reconnect / A2L reload: old counts don't carry over
    int     m_crossCheckSamples          = 0;      // total samples actually compared
    int     m_crossCheckMismatches       = 0;      // of those, how many failed tolerance
    double  m_lastCrossCheckDeltaDeg     = 0.0;
    bool    m_crossCheckCurrentChecked   = false;   // was the LATEST sample compared?
    QString m_crossCheckNotCheckedReason = QStringLiteral("no sample yet");
};

#endif // ATTITUDEVIEW_H
