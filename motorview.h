#ifndef MOTORVIEW_H
#define MOTORVIEW_H

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
class QSlider;
class QSpinBox;
class QGroupBox;
class MotorBar;

// "Motors" sub-tab: ARM/DISARM, manual-throttle switch, one throttle slider
// per motor and a live readout of the Esc* measurements. Replaces hand-editing
// CAL_motorCmd / CAL_motorManual / CAL_motorManualSp_Mn in the Calibration tab.
//
// Everything is resolved BY NAME against the loaded A2L (EscMotorState,
// EscM<n>*, CAL_motor*). This widget does no XCP I/O: it emits
// armRequested/disarmRequested/writeCharRequested and XcpPanel performs the
// writes (and logs them), like SystemFooter. It never emits a write while
// disconnected, and never a manual setpoint of 1..47 (DShot commands).
//
// Firmware contract (motor.c): the firmware zeroes CAL_motorManual and
// CAL_motorManualSp_* on every exit from ARMED, so on any such exit the view
// resets its own sliders/toggle WITHOUT writing.
class MotorView : public QWidget
{
    Q_OBJECT

public:
    explicit MotorView(QWidget *parent = nullptr);

    void setAvailable(const QVector<A2lMeas> &meas);
    void setCharsAvailable(const QVector<A2lChar> &chars);

public slots:
    void setConnected(bool connected);
    void feedSample(const XcpClient::Measurements &m);
    // Read-back of CAL_motorCmd / CAL_motorManual / CAL_motorManualSp_M1..4
    // from XcpPanel, keyed by characteristic NAME. This (plus EscMotorState)
    // is the only source for the manual toggle and the slider enable state.
    // fresh = the read was issued after the last CAL_motor* write was enqueued;
    // a stale reply never confirms a pending manual on/off step.
    // A DISARM was issued (this tab's button or the footer's): drop pending slider
    // writes and lock the sliders until the firmware confirms the exit from ARMED.
    void onDisarmIssued();
    void onCharsRead(const QMap<QString, quint32> &values, bool fresh = true);
    void setReadBackProblem(const QString &reason);   // read-back window unusable (A2L layout)

signals:
    void armRequested();
    void disarmRequested();                 // XcpPanel: purge Sp writes, priority CAL_motorCmd=0
    void stopAllRequested();                // XcpPanel: purge Sp writes, priority Sp=0 x4
    void writeCharRequested(const QString &name, quint32 value);   // setpoint writes are queue-coalesced
    void readBackRequested();               // ask XcpPanel for a CAL_motor* read-back
    void logMessage(const QString &text);

private:
    static constexpr int kMotors = 4;

    struct MotorIdx {
        int alive = -1, dshot = -1, bidirRpm = -1, bidirErpm = -1, bidirOk = -1,
            rpm = -1, erpm100 = -1, temp = -1, tlmCount = -1, tlmMissed = -1,
            volt = -1, cur = -1, mah = -1;
    };
    struct MotorUi {
        QGroupBox *box = nullptr;
        QLabel    *rpmLbl = nullptr, *rpmCaption = nullptr, *erpmLbl = nullptr,
                  *tempLbl = nullptr, *dshotLbl = nullptr, *countsLbl = nullptr,
                  *aliveLamp = nullptr, *aliveLbl = nullptr,
                  *rawLbl = nullptr,
                  *kRpmLbl = nullptr, *kErpmLbl = nullptr, *tlmLbl = nullptr;
        MotorBar  *bar = nullptr;
        QSlider   *slider = nullptr;
        QSpinBox  *spin = nullptr;
    };

    enum class Manual { Unknown, Off, On };                 // as read back from the firmware
    enum class Pending { None, AwaitZeros, AwaitOn, AwaitOff };   // operator request in flight

    void applyCeiling(quint32 value, bool fresh);
    void applyRange();                    // slider / spinbox maximum from A2L limit + ceiling
    void onSetCeiling();
    void updateCeilingUi();
    void onSliderChanged(int motor, int pos);
    void onSpinChanged(int motor, int pos);
    void onArmClicked();
    void onManualClicked(bool on);
    void onStopAll();
    void flushPending();
    void resetManualUi();                 // GUI-only reset, never writes
    void writeAllStop();
    void setSliderSilently(int motor, int pos);
    void updateControls();
    void updateBanner();
    void showReadoutsStale();
    void checkStale();
    void pendingTimeout();
    void abortPending(const QString &why);
    bool slidersEnabled() const;
    bool isStale() const;
    bool manualWritable() const;
    QString disabledReason() const;

    MotorUi   m_ui[kMotors];
    MotorIdx  m_idx[kMotors];
    QLabel      *m_stateLbl = nullptr, *m_linkLamp = nullptr, *m_linkLbl = nullptr,
                *m_hintLbl = nullptr, *m_bindLbl = nullptr;
    QPushButton *m_armBtn = nullptr, *m_disarmBtn = nullptr, *m_stopAllBtn = nullptr,
                *m_manualBtn = nullptr;
    QCheckBox   *m_linkAll = nullptr;
    QGroupBox   *m_rawBox = nullptr;

    QVector<A2lMeas> m_meas;
    int  m_idxState = -1;
    bool m_charsOk  = false;     // CAL_motorCmd + CAL_motorManual + Sp_M1..4 all in the A2L
    bool m_cmdOk    = false;     // CAL_motorCmd alone (ARM/DISARM)
    int  m_spHi     = 2047;      // upper limit of CAL_motorManualSp_Mn from the A2L
    int  m_spMax    = 2047;      // effective slider maximum: min(m_spHi, ceiling read-back)
    bool m_ceilOk   = false;     // CAL_motorSpMax is in the A2L (older firmware: false)
    int  m_ceiling  = -1;        // CAL_motorSpMax as read back from the firmware, -1 unknown
    int  m_ceilReq  = -1;        // value of an unconfirmed "Set ceiling" write, -1 none
    QLabel      *m_ceilLbl = nullptr;
    QSpinBox    *m_ceilSpin = nullptr;
    QPushButton *m_ceilBtn = nullptr;
    QTimer       m_ceilTimer;    // timeout of an unconfirmed ceiling write

    bool m_connected  = false;
    bool m_haveSample = false;
    int  m_state      = -1;      // EscMotorState, -1 unknown
    Manual  m_manual  = Manual::Off;       // from read-back only, never from our own write
    Pending m_pendingOp = Pending::None;
    QString m_readBackProblem;             // non-empty: manual ON can never be confirmed
    QString m_notice;                      // why the tab changed its mind (shown in the hint)
    bool    m_locked  = false;             // DISARM sent, waiting for the firmware to confirm
    bool m_inLink     = false;   // re-entrancy guard for "link all four"

    int  m_pending[kMotors] = { -1, -1, -1, -1 };   // slider pos waiting for the coalescing timer
    QTimer m_flushTimer;
    QTimer m_staleTimer;
    QTimer m_pollTimer;      // 1 Hz CAL_motorManual read-back while ARMED
    QTimer m_pendTimer;      // timeout of a pending manual on/off
    QTimer m_lockTimer;      // releases m_locked if the DISARM never lands
    QElapsedTimer m_sinceSample;
};

#endif // MOTORVIEW_H
