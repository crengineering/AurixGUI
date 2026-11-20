#ifndef CTRLVIEW_H
#define CTRLVIEW_H

#include <QWidget>
#include <QVector>
#include <QString>
#include <QMap>
#include <QElapsedTimer>
#include <QTimer>
#include "a2lmodel.h"
#include "xcpclient.h"

class QLabel;
class QPushButton;
class QCheckBox;
class QDoubleSpinBox;
class QGroupBox;
class PlotWidget;

// Flight-controller operation, part of the "Attitude" tab: attitude setpoints
// (roll / pitch / yaw), Shadow / Drive switch, collective thrust, and the
// immediate feedback (setpoint vs actual as numbers and as three short live
// plots, controller request vs measured rpm, CtrlStatus / CtrlTrips).
//
// Everything is resolved BY NAME against the loaded A2L (CAL_ctrl*, Ctrl*,
// Att*, Esc*). This widget does no XCP I/O: it emits writeCharRequested() /
// readBackRequested() and XcpPanel performs and logs them, like MotorView.
//
// Firmware contract (A2L CAL_ctrl* descriptions): CAL_ctrlMode and the setpoints
// are only kept while ARMED; the firmware resets them in every other state and
// when the controller request is unusable (a "trip"). So nothing here is
// believed until read back: the values shown as "in force" are the read-back,
// never the typed text, and the entry is disabled with a visible reason unless
// the vehicle is ARMED.
class CtrlView : public QWidget
{
    Q_OBJECT

public:
    explicit CtrlView(QWidget *parent = nullptr);

    void setAvailable(const QVector<A2lMeas> &meas);
    void setCharsAvailable(const QVector<A2lChar> &chars);

public slots:
    void setConnected(bool connected);
    void feedSample(const XcpClient::Measurements &m);
    // Read-back of the CAL_ctrl* words by name. fresh = the read was issued
    // after the last CAL_ctrl* write was enqueued (a stale reply never confirms
    // a pending request).
    void onCharsRead(const QMap<QString, double> &values, bool fresh);
    // CAL_motorManual from the Motors tab's read-back: manual wins over the controller.
    void onMotorCharsRead(const QMap<QString, quint32> &values, bool fresh);

signals:
    void writeCharRequested(const QString &name, double value);
    void readBackRequested();
    void logMessage(const QString &text);

private:
    struct AxisUi {
        QLabel *sp = nullptr, *act = nullptr, *err = nullptr,
               *omSoll = nullptr, *tau = nullptr, *tauI = nullptr;
        PlotWidget *plot = nullptr;
        int serSp = -1, serAct = -1;
    };
    struct MotorUi {
        QLabel *req = nullptr, *meas = nullptr, *diff = nullptr;
    };

    void buildUi();
    void applyFirmwareWord(const QString &name, double v);
    void applyRanges();
    void syncEntryFromReadBack();
    void updateControls();
    void updateInForce();
    void showReadoutsStale();
    void checkStale();
    bool isStale() const;
    bool armed() const { return m_state == 2; }
    QString entryDisabledReason() const;
    QString driveDisabledReason() const;
    QString thrustDisabledReason() const;
    bool modeKnown() const;
    void noteModeChange(int newMode);

    void onApply();
    void onLevel();
    void onHoldToggled(bool hold);
    void onSetThrust();
    void onShadowClicked();
    void onDriveClicked();
    void noteTrip(quint32 status, quint32 trips);
    void setNotice(const QString &text);
    void writeWord(const QString &name, double value);

    // ---- A2L binding ------------------------------------------------------
    QVector<A2lMeas> m_meas;
    QVector<A2lChar> m_chars;
    int m_idxState = -1, m_idxStatus = -1, m_idxTrips = -1, m_idxMode = -1, m_idxTripReason = -1;
    int m_idxAtt[3] = { -1, -1, -1 }, m_idxSp[3] = { -1, -1, -1 };
    int m_idxOm[3] = { -1, -1, -1 }, m_idxTau[3] = { -1, -1, -1 }, m_idxTauI[3] = { -1, -1, -1 };
    int m_idxCtrlRpm[4] = { -1, -1, -1, -1 }, m_idxRpm[4] = { -1, -1, -1, -1 }, m_idxRpmOk[4] = { -1, -1, -1, -1 };
    bool m_modeOk = false;       // CAL_ctrlMode in the A2L
    bool m_spOk = false;         // CAL_ctrlAttSp_0..2 in the A2L
    bool m_thrOk = false;        // CAL_ctrlThrustN in the A2L
    double m_thrHi = 40.0;       // A2L upper limit of CAL_ctrlThrustN

    // ---- firmware read-back (never our own write) -------------------------
    QMap<QString, double> m_cal;
    QElapsedTimer m_calAge;      // time of the newest read-back
    bool m_haveCal = false;
    int  m_manual = -1;          // CAL_motorManual as read back, -1 unknown

    // ---- operator requests in flight --------------------------------------
    int    m_modeReq = -1;
    bool   m_spPending = false;
    double m_spReq[3] = { 0, 0, -1 };
    double m_thrReq = qQNaN();
    QTimer m_modeTimer, m_spTimer, m_thrTimer;
    QString m_notice;            // why the controller or a request is not in the state the operator asked for

    // ---- measurements ------------------------------------------------------
    bool m_connected = false;
    bool m_haveSample = false;
    int  m_state = -1;
    quint32 m_trips = 0;
    bool m_haveTrips = false;
    quint32 m_faultSeen = 0;     // fault bits seen since the last trip report (fallback when CtrlTripReason is not in the A2L)
    quint32 m_tripReason = 0;    // CtrlTripReason, latched by the firmware
    int  m_prevState = -1;
    int  m_prevMode = -1;        // last read-back mode, for "dropped by the firmware"
    QElapsedTimer m_sinceSample;
    QElapsedTimer m_tripClock;   // valid once a trip was reported (suppresses the duplicate mode-drop notice)
    bool m_entryDirty = false;   // typed but not yet applied: the read-back must not overwrite it
    QElapsedTimer m_plotClock;
    QTimer m_staleTimer;
    double m_yawPrevRaw = qQNaN(), m_yawUnwrapped = 0.0;

    // ---- widgets -----------------------------------------------------------
    QLabel *m_banner = nullptr, *m_bindLbl = nullptr, *m_stateLbl = nullptr, *m_modeLbl = nullptr,
           *m_statusLbl = nullptr, *m_tripsLbl = nullptr, *m_reasonLbl = nullptr, *m_driveReasonLbl = nullptr,
           *m_thrInForce = nullptr, *m_thrReasonLbl = nullptr, *m_limitLbl = nullptr;
    QPushButton *m_shadowBtn = nullptr, *m_driveBtn = nullptr, *m_applyBtn = nullptr,
                *m_levelBtn = nullptr, *m_thrBtn = nullptr;
    QDoubleSpinBox *m_spin[3] = { nullptr, nullptr, nullptr };
    QDoubleSpinBox *m_thrSpin = nullptr;
    QCheckBox *m_hold = nullptr;
    QLabel *m_inForce[3] = { nullptr, nullptr, nullptr };
    QGroupBox *m_spBox = nullptr, *m_thrBox = nullptr;
    AxisUi  m_axis[3];
    MotorUi m_mot[4];
};

#endif // CTRLVIEW_H
