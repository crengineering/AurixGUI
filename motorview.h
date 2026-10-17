#ifndef MOTORVIEW_H
#define MOTORVIEW_H

#include <QWidget>
#include <QVector>
#include <QString>
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

signals:
    void armRequested();
    void disarmRequested();
    void writeCharRequested(const QString &name, quint32 value);

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
    int  m_spMax    = 2047;      // upper limit of CAL_motorManualSp_Mn from the A2L

    bool m_connected  = false;
    bool m_haveSample = false;
    int  m_state      = -1;      // EscMotorState, -1 unknown
    bool m_manual     = false;   // our belief, set by our own manual write
    bool m_inLink     = false;   // re-entrancy guard for "link all four"

    int  m_pending[kMotors] = { -1, -1, -1, -1 };   // slider pos waiting for the coalescing timer
    QTimer m_flushTimer;
    QTimer m_staleTimer;
    QElapsedTimer m_sinceSample;
};

#endif // MOTORVIEW_H
