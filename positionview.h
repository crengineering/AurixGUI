#ifndef POSITIONVIEW_H
#define POSITIONVIEW_H

#include <QWidget>
#include <QVector>
#include <QElapsedTimer>
#include "a2lmodel.h"
#include "xcpclient.h"
#include "positionmath.h"

class QLabel;
class QComboBox;
class PositionTrailWidget;
class AltitudeBarWidget;

// "Position" panel beside the Attitude view (SYS1-016 -> SYS2-GUI-003 ->
// SWE1-GUI-006/-007/-008): top-down N/E trail with a velocity arrow and
// speed label at the current-position marker, an altitude bar with a
// vertical-rate readout, text readouts and the anchoring state. Fed by the
// SAME XCP DAQ sample as AttitudeView -- no second XCP session, no I/O in
// the paint path -- mirroring its setAvailable()/setConnected()/
// feedSample() delivery path exactly.
//
// Channels are resolved by NAME against the loaded A2L (NavPosNorth/East/
// Down, NavVelNorth/East/Down, NavVerticalOk, NavHorizontalOk, NavOriginSet,
// GnssNavOk, GnssFixType, GnssNumSats, GnssHAccuracy, and AttState for the
// same live/frozen gate the attitude view uses); if the three position
// channels are missing the panel says so and stays inert, same rule as
// AttitudeView. NavGnssTrusted / NavStationaryLocked (SWE1-GUI-007
// amendment, fw >= 1.19.26) are OPTIONAL: an older A2L without them falls
// back to the pre-amendment two-way anchoring text, tagged "[legacy]" so the
// missing visibility is visible rather than silently reused.
class PositionView : public QWidget
{
    Q_OBJECT

public:
    explicit PositionView(QWidget *parent = nullptr);

    // Re-resolve the channels against a (possibly new) A2L MEASUREMENT list.
    void setAvailable(const QVector<A2lMeas> &meas);

public slots:
    // Mirrors XcpPanel's own connected/disconnected state -- never presents
    // the last position as live once the link is down.
    void setConnected(bool connected);

    // One decoded DAQ sample, already assembled by XcpClient -- no I/O here,
    // a pure in-memory decode of bytes that already arrived.
    void feedSample(const XcpClient::Measurements &m);

private slots:
    void onWindowChanged(int index);

private:
    struct TrailPoint { double north; double east; qint64 tMs; };

    void updateBoundState();
    void showFrozenReadout();
    void pruneTrail(qint64 nowMs);
    void resetTrail();
    // Frame description in the panel's intro line: "relative to the
    // tangent-plane origin" once NavOriginSet, else "relative to power-on /
    // dead reckoning" -- (0,0) means something different in each case, and
    // showing an origin marker while the readout says "not set" would
    // contradict itself (review finding MINOR 6).
    void updateIntroText(bool haveOrigin);

    PositionTrailWidget *m_trail       = nullptr;
    AltitudeBarWidget   *m_altBar      = nullptr;
    QComboBox           *m_windowBox   = nullptr;
    QLabel              *m_introLbl    = nullptr;   // origin-relative vs power-on wording
    QLabel *m_nLbl = nullptr, *m_eLbl = nullptr, *m_upLbl = nullptr;
    QLabel *m_velLbl        = nullptr;
    QLabel *m_horizLbl      = nullptr;
    QLabel *m_vertLbl       = nullptr;
    QLabel *m_originLbl     = nullptr;
    QLabel *m_stateLbl      = nullptr;
    QLabel *m_lockLbl       = nullptr;   // "LOCK" badge, NavStationaryLocked (SWE1-GUI-007 amendment)
    QLabel *m_bindStatusLbl = nullptr;

    QVector<A2lMeas> m_meas;
    int m_idxPosN = -1, m_idxPosE = -1, m_idxPosD = -1;
    int m_idxVelN = -1, m_idxVelE = -1, m_idxVelD = -1;
    int m_idxVertOk = -1, m_idxHorizOk = -1, m_idxOriginSet = -1;
    int m_idxGnssNavOk = -1, m_idxGnssFixType = -1, m_idxGnssNumSats = -1, m_idxGnssHAcc = -1;
    int m_idxGnssTrusted = -1, m_idxStationaryLocked = -1;   // optional (SWE1-GUI-007 amendment)
    int m_idxAttState = -1;
    bool m_bound     = false;   // NavPosNorth/East/Down all resolved
    bool m_connected = false;

    QVector<TrailPoint> m_trailPts;         // oldest .. newest
    QElapsedTimer        m_clock;
    // Trail length is kPosTrailLengthMs (positionmath.h) -- ONE definition,
    // shared with PositionTrailWidget's age-fade divisor (review finding
    // MINOR 7 was the two silently disagreeing).
    PosWindow m_window = PosWindow::W5m;    // design default: 5 m
};

#endif // POSITIONVIEW_H
