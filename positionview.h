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
// SWE1-GUI-006/-007): top-down N/E trail, altitude bar, readouts and the
// anchoring state. Fed by the SAME XCP DAQ sample as AttitudeView -- no
// second XCP session, no I/O in the paint path -- mirroring its
// setAvailable()/setConnected()/feedSample() delivery path exactly.
//
// Channels are resolved by NAME against the loaded A2L (NavPosNorth/East/
// Down, NavVelNorth/East/Down, NavVerticalOk, NavHorizontalOk, NavOriginSet,
// GnssNavOk, GnssFixType, GnssNumSats, GnssHAccuracy, and AttState for the
// same live/frozen gate the attitude view uses); if the three position
// channels are missing the panel says so and stays inert, same rule as
// AttitudeView.
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

    PositionTrailWidget *m_trail       = nullptr;
    AltitudeBarWidget   *m_altBar      = nullptr;
    QComboBox           *m_windowBox   = nullptr;
    QLabel *m_nLbl = nullptr, *m_eLbl = nullptr, *m_upLbl = nullptr;
    QLabel *m_velLbl        = nullptr;
    QLabel *m_horizLbl      = nullptr;
    QLabel *m_vertLbl       = nullptr;
    QLabel *m_originLbl     = nullptr;
    QLabel *m_stateLbl      = nullptr;
    QLabel *m_bindStatusLbl = nullptr;

    QVector<A2lMeas> m_meas;
    int m_idxPosN = -1, m_idxPosE = -1, m_idxPosD = -1;
    int m_idxVelN = -1, m_idxVelE = -1, m_idxVelD = -1;
    int m_idxVertOk = -1, m_idxHorizOk = -1, m_idxOriginSet = -1;
    int m_idxGnssNavOk = -1, m_idxGnssNumSats = -1, m_idxGnssHAcc = -1;
    int m_idxAttState = -1;
    bool m_bound     = false;   // NavPosNorth/East/Down all resolved
    bool m_connected = false;

    QVector<TrailPoint> m_trailPts;         // oldest .. newest
    QElapsedTimer        m_clock;
    // Design default (dispatch/SYS1-016 recommendation, stated in the
    // aurix-gui report and SWE1-GUI-006): the trail keeps the last 60 s, not
    // a user setting.
    static constexpr qint64 kTrailLengthMs = 60000;
    PosWindow m_window = PosWindow::W5m;    // design default: 5 m
};

#endif // POSITIONVIEW_H
