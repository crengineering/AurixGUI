#ifndef SYSTEMFOOTER_H
#define SYSTEMFOOTER_H

#include <QWidget>
#include <QVector>
#include "xcpclient.h"
#include "a2lmodel.h"

class QLabel;
class QProgressBar;
class QPushButton;

/*
 * Permanent status strip along the bottom of the main window: execution time
 * for all six TriCores, the Ethernet link utilisation and a presence lamp per
 * peripheral sensor.
 *
 * It lives below the tab widget rather than inside a tab, so it stays visible
 * whichever tab is open - the point of a load display is that you see it while
 * doing something else.
 *
 * Per core it shows the busy time measured in the last 100 ms window and that
 * time as a percentage. A core whose alive counter stops incrementing is drawn
 * in red: that is a hung core, which otherwise looks identical to an idle one
 * (both report 0 % load).
 */
class SystemFooter : public QWidget
{
    Q_OBJECT

public:
    // IMU, Mag, Baro, GNSS - the peripherals with a presence lamp. Public so
    // the table describing them in the .cpp can be checked against it.
    static constexpr int SensorCount = 4;

    // M1..M4 ESC telemetry-alive lamps. Public for the same reason as
    // SensorCount.
    static constexpr int ActuatorCount = 4;

    explicit SystemFooter(QWidget *parent = nullptr);

    // Re-resolve the ESC alive/state signals against a (possibly new) A2L
    // MEASUREMENT list. Mirrors AttitudeView::setAvailable() / PlotPane's: the
    // footer never hardcodes an address, and an A2L that lacks the Esc*
    // signals (older firmware) just leaves the lamps at "unknown" instead of
    // misreading whatever now sits at a stale offset.
    void setAvailable(const QVector<A2lMeas> &meas);

    // Re-resolve CAL_motorCmd against a (possibly new) A2L CHARACTERISTIC
    // list. Same "unknown until the A2L says otherwise" rule as setAvailable():
    // an A2L that does not describe CAL_motorCmd (older firmware) leaves ARM/
    // DISARM disabled instead of writing to whatever now sits at a stale
    // address.
    void setCharsAvailable(const QVector<A2lChar> &chars);

signals:
    // ARM/DISARM button clicks, already past the confirmation for ARM. The
    // footer only asks "should this happen" - resolving CAL_motorCmd's
    // address and doing the actual XCP write is XcpPanel's job, the same
    // place every other characteristic write and its error reporting live.
    void armRequested();
    void disarmRequested();

public slots:
    void update(const XcpClient::Measurements &m);
    void setConnected(bool connected);

private slots:
    void onArmClicked();
    void onDisarmClicked();

private:
    void updateArmDisarmEnabled();
    // What the firmware's "present"/"alive" flag currently says. Unknown is
    // not the same as Absent: with no XCP connection (or a signal missing
    // from the loaded A2L) nothing has actually been read, and an unlit lamp
    // there would claim the part is missing/dead rather than merely unknown.
    enum class Presence { Unknown, Absent, Present };

    void setPresence(int i, Presence p);
    void setActuatorPresence(int i, Presence p);

    QLabel       *m_coreName[6] = {};
    QLabel       *m_coreValue[6] = {};
    QProgressBar *m_ethBar      = nullptr;
    QLabel       *m_ethText     = nullptr;

    // One lamp + name per peripheral, in kSensors[] order.
    QLabel       *m_sensorLamp[SensorCount] = {};
    QLabel       *m_sensorName[SensorCount] = {};

    // One lamp + name per ESC, in kActuators[] order, plus a small text label
    // for EscMotorState next to them.
    QLabel       *m_actuatorLamp[ActuatorCount] = {};
    QLabel       *m_actuatorName[ActuatorCount] = {};
    QLabel       *m_actuatorStateVal            = nullptr;

    // ARM/DISARM: writes CAL_motorCmd (1 = armed, 0 = disarmed) through
    // XcpPanel. Enabled only while connected and while the loaded A2L
    // describes CAL_motorCmd.
    QPushButton  *m_armBtn    = nullptr;
    QPushButton  *m_disarmBtn = nullptr;

    // Signals resolved by name against the loaded A2L, same pattern as
    // AttitudeView/PlotPane. -1 = not described by this A2L (older firmware).
    QVector<A2lMeas> m_meas;
    int m_idxEscAlive[ActuatorCount] = { -1, -1, -1, -1 };
    int m_idxEscState                = -1;

    // CHARACTERISTIC list, resolved the same way (setCharsAvailable()).
    int m_idxMotorCmd = -1;

    quint16 m_lastAlive[6]  = {0, 0, 0, 0, 0, 0};
    int     m_staleCount[6] = {0, 0, 0, 0, 0, 0};
    bool    m_connected     = false;
};

#endif // SYSTEMFOOTER_H
