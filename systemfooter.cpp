#include "systemfooter.h"
#include "appicon.h"
#include "lampicon.h"

#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QHBoxLayout>
#include <QFrame>
#include <QMessageBox>

namespace {
/* Windows before a non-incrementing alive counter is called a hung core. The
 * firmware bumps it every 100 ms and the GUI polls slower, so a couple of
 * repeats are normal; three in a row are not. */
constexpr int STALE_LIMIT = 3;

/* Bead diameter of the sensor bulbs - the same 12 px as the tab-bar lamps.
 * The halo lives in the margin bulbPixmap() adds around it. */
constexpr int BULB_BEAD = 12;

/* The peripherals the footer watches, and the part that has to answer when the
 * firmware probes the bus at start-up. Order matches m_sensorLamp[]. */
struct SensorDef {
    const char *label;
    const char *chip;
};
constexpr SensorDef kSensors[] = {
    { "IMU",  "ICM-42688-P" },
    { "Mag",  "MMC5983MA"   },
    { "Baro", "BMP581"      },
    { "GNSS", "NEO-M9N"     },
};
static_assert(sizeof(kSensors) / sizeof(kSensors[0]) == SystemFooter::SensorCount,
              "kSensors and the lamp arrays must describe the same sensors");

// The four ESCs, in EscM<n>Alive order. Order matches m_actuatorLamp[].
constexpr const char *kActuators[] = { "M1", "M2", "M3", "M4" };
static_assert(sizeof(kActuators) / sizeof(kActuators[0]) == SystemFooter::ActuatorCount,
              "kActuators and the lamp arrays must describe the same ESCs");

int indexOfMeas(const QVector<A2lMeas> &meas, const char *name)
{
    for (int i = 0; i < meas.size(); ++i)
        if (meas[i].name == QLatin1String(name))
            return i;
    return -1;
}

int indexOfChar(const QVector<A2lChar> &chars, const char *name)
{
    for (int i = 0; i < chars.size(); ++i)
        if (chars[i].name == QLatin1String(name))
            return i;
    return -1;
}

QString escMotorStateText(int state)
{
    switch (state) {
    case 0: return QStringLiteral("DISARMED");
    case 1: return QStringLiteral("INIT");
    case 2: return QStringLiteral("ARMED");
    case 3: return QStringLiteral("FAILSAFE");
    default: return QStringLiteral("--");
    }
}

QFrame *makeSeparator()
{
    auto *line = new QFrame;
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Sunken);
    // Colour set here, not as a "QFrame" rule in the strip's stylesheet: QLabel
    // derives from QFrame, so such a rule matches every label too - and being
    // the later rule of two equally specific ones, it won. That is what had all
    // the footer text rendering at 35 % alpha instead of its own colour.
    line->setStyleSheet("color: rgba(0, 0, 0, 0.35);");
    return line;
}
} // namespace

SystemFooter::SystemFooter(QWidget *parent)
    : QWidget(parent)
{
    // WA_StyledBackground: a QWidget subclass ignores a stylesheet background
    // unless it either paints one itself or this attribute is set.
    setAttribute(Qt::WA_StyledBackground, true);
    // Black text rather than the near-white the badge uses elsewhere: the light
    // labels washed out against the red strip. The progress bar is inverted
    // along with them - its fill and border were dark so that white text stayed
    // readable across it, which is now exactly backwards.
    setStyleSheet(QString(
        "SystemFooter { background: %1; }"
        "QLabel { color: #000000; }"
        "QProgressBar { border: 1px solid rgba(0, 0, 0, 0.55);"
        " border-radius: 5px; background: rgba(255, 255, 255, 0.18);"
        " color: #000000; text-align: center; }"
        // Light fill, the mirror of the old dark one: the percentage is drawn
        // across the whole bar, so both halves have to stay readable in black.
        "QProgressBar::chunk { background: rgba(255, 255, 255, 0.38); border-radius: 4px; }")
        .arg(AppIcon::Background.name()));

    auto *row = new QHBoxLayout(this);
    row->setContentsMargins(6, 2, 6, 2);
    row->setSpacing(6);

    auto *coresLabel = new QLabel(tr("Cores:"));
    QFont bold = coresLabel->font();
    bold.setBold(true);
    coresLabel->setFont(bold);
    row->addWidget(coresLabel);

    QFont mono("Consolas");
    mono.setStyleHint(QFont::Monospace);
    mono.setPointSize(9);

    for (int i = 0; i < 6; ++i) {
        m_coreName[i] = new QLabel(QString("C%1").arg(i));
        m_coreName[i]->setFont(mono);
        m_coreValue[i] = new QLabel("--");
        m_coreValue[i]->setFont(mono);
        m_coreValue[i]->setMinimumWidth(96);
        m_coreValue[i]->setToolTip(tr("Busy time in the last 100 ms window, "
                                      "and that time as a percentage.\n"
                                      "Red = the core's alive counter stopped: it is hung."));
        row->addWidget(m_coreName[i]);
        row->addWidget(m_coreValue[i]);
        if (i < 5)
            row->addWidget(makeSeparator());
    }

    row->addWidget(makeSeparator());

    auto *ethLabel = new QLabel(tr("Ethernet:"));
    ethLabel->setFont(bold);
    row->addWidget(ethLabel);

    m_ethBar = new QProgressBar;
    m_ethBar->setRange(0, 100);            // percent of the negotiated link rate
    m_ethBar->setValue(0);
    m_ethBar->setTextVisible(true);
    m_ethBar->setFormat("%p%");
    m_ethBar->setFixedWidth(160);
    m_ethBar->setToolTip(tr("Link utilisation: TX+RX throughput as a percentage "
                            "of the negotiated line rate."));
    row->addWidget(m_ethBar);

    m_ethText = new QLabel("--");
    m_ethText->setFont(mono);
    m_ethText->setMinimumWidth(150);
    row->addWidget(m_ethText);

    row->addWidget(makeSeparator());

    // Peripheral presence. A lamp rather than a number because that is all the
    // firmware publishes: each driver probes its part once at init and sets a
    // single "it answered" flag. Without it a sensor that was never detected
    // and one that is genuinely reading zero look exactly the same.
    auto *sensorsLabel = new QLabel(tr("Sensors:"));
    sensorsLabel->setFont(bold);
    row->addWidget(sensorsLabel);

    for (int i = 0; i < SensorCount; ++i) {
        m_sensorLamp[i] = new QLabel;
        m_sensorName[i] = new QLabel(QString::fromLatin1(kSensors[i].label));
        m_sensorName[i]->setFont(mono);

        // Lamp and name are one cell: the row spacing separates the sensors
        // from each other, the tighter cell spacing ties each lamp to its label.
        auto *cell = new QHBoxLayout;
        cell->setSpacing(3);
        cell->addWidget(m_sensorLamp[i]);
        cell->addWidget(m_sensorName[i]);
        row->addLayout(cell);
    }

    row->addWidget(makeSeparator());

    // ESC telemetry-alive. Same lamp widget as Sensors, but per-cycle DAQ data
    // rather than an init-time probe: the driver clears EscM<n>Alive after 3
    // consecutive missed telemetry requests, so an unlit lamp here means the
    // link to that ESC is currently down, not merely "never detected".
    auto *actuatorsLabel = new QLabel(tr("Actuators:"));
    actuatorsLabel->setFont(bold);
    row->addWidget(actuatorsLabel);

    for (int i = 0; i < ActuatorCount; ++i) {
        m_actuatorLamp[i] = new QLabel;
        m_actuatorName[i] = new QLabel(QString::fromLatin1(kActuators[i]));
        m_actuatorName[i]->setFont(mono);

        auto *cell = new QHBoxLayout;
        cell->setSpacing(3);
        cell->addWidget(m_actuatorLamp[i]);
        cell->addWidget(m_actuatorName[i]);
        row->addLayout(cell);
    }

    // EscMotorState next to the lamps: the one word that says whether the
    // lamps below even mean anything yet (no telemetry is expected while
    // still DISARMED).
    m_actuatorStateVal = new QLabel("--");
    m_actuatorStateVal->setFont(mono);
    m_actuatorStateVal->setMinimumWidth(72);
    m_actuatorStateVal->setToolTip(tr("EscMotorState: DISARMED / INIT / ARMED / FAILSAFE."));
    row->addWidget(m_actuatorStateVal);

    // ARM/DISARM, right of the state text: writes CAL_motorCmd. ARM asks for
    // confirmation (arming is deliberate); DISARM never does - it is the
    // software kill switch and has to fire on the first click. Both styled
    // as solid white buttons rather than left in the footer's default look:
    // against the red strip a merely-outlined button all but disappears, and
    // DISARM in particular has to be the first thing the eye finds here.
    m_armBtn = new QPushButton(tr("ARM"));
    m_armBtn->setFont(mono);
    m_armBtn->setToolTip(tr("Arm the motors (CAL_motorCmd = 1). Asks for "
                            "confirmation first."));
    m_armBtn->setStyleSheet(
        "QPushButton { background: rgba(255, 255, 255, 0.85); color: #000000;"
        " border: 1px solid rgba(0, 0, 0, 0.55); border-radius: 4px;"
        " padding: 2px 10px; }"
        "QPushButton:hover:!disabled { background: rgba(255, 255, 255, 1.0); }"
        "QPushButton:disabled { background: rgba(255, 255, 255, 0.35);"
        " color: rgba(0, 0, 0, 0.45); }");
    row->addWidget(m_armBtn);

    m_disarmBtn = new QPushButton(tr("DISARM"));
    QFont disarmFont = mono;
    disarmFont.setBold(true);
    m_disarmBtn->setFont(disarmFont);
    m_disarmBtn->setToolTip(tr("Disarm the motors immediately (CAL_motorCmd = 0). "
                               "No confirmation - this is the kill switch."));
    m_disarmBtn->setStyleSheet(
        "QPushButton { background: #ffffff; color: #b22228;"
        " border: 2px solid #b22228; border-radius: 4px; padding: 2px 10px; }"
        "QPushButton:hover:!disabled { background: #ffe8e8; }"
        "QPushButton:disabled { background: rgba(255, 255, 255, 0.35);"
        " color: rgba(178, 34, 40, 0.45); border-color: rgba(178, 34, 40, 0.45); }");
    row->addWidget(m_disarmBtn);

    connect(m_armBtn,    &QPushButton::clicked, this, &SystemFooter::onArmClicked);
    connect(m_disarmBtn, &QPushButton::clicked, this, &SystemFooter::onDisarmClicked);

    row->addStretch(1);
    setConnected(false);   // also disables ARM/DISARM until connected
}

void SystemFooter::setPresence(int i, Presence p)
{
    const bool lit = (p == Presence::Present);
    const QColor color = lit                     ? LampColor::Green
                       : (p == Presence::Absent) ? LampColor::Off
                                                 : LampColor::Gray;
    m_sensorLamp[i]->setPixmap(bulbPixmap(color, lit, BULB_BEAD));

    // Dim the name as well. Lit against unlit is a small difference at this
    // size, and the label is the bigger target for a glance across the strip.
    // Only to 65 % though: dimming is a hint, and a label nobody can read is
    // worse than one that does not stand out.
    //
    // Both branches name a colour instead of the lit one clearing its
    // stylesheet and inheriting the strip's: the two states are then set the
    // same way and read side by side here, rather than one of them depending
    // on whatever the strip happens to cascade down.
    m_sensorName[i]->setStyleSheet(lit ? "color: #000000;"
                                       : "color: rgba(0, 0, 0, 0.65);");

    const QString state = lit                     ? tr("detected")
                        : (p == Presence::Absent) ? tr("NOT detected")
                                                  : tr("unknown - not connected");
    const QString tip = tr("%1 (%2): %3\n"
                           "Lit = the part answered when the firmware probed it "
                           "at start-up.")
                            .arg(QString::fromLatin1(kSensors[i].label),
                                 QString::fromLatin1(kSensors[i].chip), state);
    m_sensorLamp[i]->setToolTip(tip);
    m_sensorName[i]->setToolTip(tip);
}

void SystemFooter::setActuatorPresence(int i, Presence p)
{
    const bool lit = (p == Presence::Present);
    const QColor color = lit                     ? LampColor::Green
                       : (p == Presence::Absent) ? LampColor::Off
                                                 : LampColor::Gray;
    m_actuatorLamp[i]->setPixmap(bulbPixmap(color, lit, BULB_BEAD));

    m_actuatorName[i]->setStyleSheet(lit ? "color: #000000;"
                                         : "color: rgba(0, 0, 0, 0.65);");

    const QString state = lit                     ? tr("alive")
                        : (p == Presence::Absent) ? tr("NOT answering")
                                                  : tr("unknown - not connected, "
                                                       "or EscM%1Alive not in this A2L").arg(i + 1);
    const QString tip = tr("ESC %1: %2\n"
                           "Lit = the DShot driver received a valid telemetry "
                           "reply recently.\nOff = 3 consecutive telemetry "
                           "requests went unanswered.")
                            .arg(QString::fromLatin1(kActuators[i]), state);
    m_actuatorLamp[i]->setToolTip(tip);
    m_actuatorName[i]->setToolTip(tip);
}

void SystemFooter::setAvailable(const QVector<A2lMeas> &meas)
{
    m_meas = meas;
    m_idxEscAlive[0] = indexOfMeas(meas, "EscM1Alive");
    m_idxEscAlive[1] = indexOfMeas(meas, "EscM2Alive");
    m_idxEscAlive[2] = indexOfMeas(meas, "EscM3Alive");
    m_idxEscAlive[3] = indexOfMeas(meas, "EscM4Alive");
    m_idxEscState    = indexOfMeas(meas, "EscMotorState");

    // A stale reading from a previous (or absent) A2L must not linger as if
    // it were still current.
    for (int i = 0; i < ActuatorCount; ++i)
        setActuatorPresence(i, Presence::Unknown);
    m_actuatorStateVal->setText("--");
}

void SystemFooter::setCharsAvailable(const QVector<A2lChar> &chars)
{
    m_idxMotorCmd = indexOfChar(chars, "CAL_motorCmd");
    updateArmDisarmEnabled();
}

void SystemFooter::updateArmDisarmEnabled()
{
    const bool enabled = m_connected && m_idxMotorCmd >= 0;
    m_armBtn->setEnabled(enabled);
    m_disarmBtn->setEnabled(enabled);
    const QString why = m_idxMotorCmd < 0
        ? tr(" (CAL_motorCmd not in the loaded A2L)")
        : (!m_connected ? tr(" (not connected)") : QString());
    if (!why.isEmpty()) {
        m_armBtn->setToolTip(tr("Arm the motors (CAL_motorCmd = 1).") + why);
        m_disarmBtn->setToolTip(tr("Disarm the motors immediately (CAL_motorCmd = 0).") + why);
    }
}

void SystemFooter::onArmClicked()
{
    const auto answer = QMessageBox::question(
        this, tr("Arm motors?"),
        tr("This arms the motors (CAL_motorCmd = 1). They will spin at any "
           "nonzero throttle command from this point on.\n\nArm?"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer == QMessageBox::Yes)
        emit armRequested();
}

void SystemFooter::onDisarmClicked()
{
    // Deliberately no confirmation: this is the kill switch, it has to act
    // on the first click.
    emit disarmRequested();
}

void SystemFooter::setConnected(bool connected)
{
    m_connected = connected;
    updateArmDisarmEnabled();
    if (!connected) {
        for (int i = 0; i < 6; ++i) {
            m_coreValue[i]->setText("--");
            m_coreValue[i]->setStyleSheet(QString());
            m_staleCount[i] = 0;
        }
        m_ethBar->setValue(0);
        m_ethText->setText("--");
        // Gray, not off: nothing has been probed, so claiming the parts are
        // missing would be a guess dressed up as a measurement.
        for (int i = 0; i < SensorCount; ++i)
            setPresence(i, Presence::Unknown);
        // Same reasoning for the ESC lamps: disconnected is not "not
        // answering", it is "no idea".
        for (int i = 0; i < ActuatorCount; ++i)
            setActuatorPresence(i, Presence::Unknown);
        m_actuatorStateVal->setText("--");
    }
}

void SystemFooter::update(const XcpClient::Measurements &m)
{
    if (!m.valid)
        return;

    for (int i = 0; i < 6; ++i) {
        // Busy time per 100 ms window. Show us below a millisecond so an idle
        // core still reads as a real number rather than rounding to 0.0 ms.
        const quint32 us = m.coreExecUs[i];
        const QString timeText = (us < 1000)
            ? QString("%1us").arg(us)
            : QString("%1ms").arg(us / 1000.0, 0, 'f', 1);

        m_coreValue[i]->setText(QString("%1 %2%")
                                    .arg(timeText, -7)
                                    .arg(m.coreLoadPmil[i] / 10.0, 4, 'f', 1));

        // A core that stops ticking its alive counter has hung. Without this a
        // dead core is indistinguishable from an idle one.
        if (m.coreAlive[i] == m_lastAlive[i]) {
            if (m_staleCount[i] < STALE_LIMIT)
                ++m_staleCount[i];
        } else {
            m_staleCount[i] = 0;
        }
        m_lastAlive[i] = m.coreAlive[i];

        // Amber, not red: the footer background is now red, so the old red
        // marker for a hung core would have been invisible against it.
        m_coreValue[i]->setStyleSheet(m_staleCount[i] >= STALE_LIMIT
                                          ? "color: #ffd400; font-weight: bold;"
                                          : QString());
    }

    const int pct = int(m.ethUtilPmil / 10);
    m_ethBar->setValue(qBound(0, pct, 100));

    // Throughput in human units next to the bar: on a gigabit link the
    // percentage is near zero for ordinary telemetry, so the raw rate is what
    // actually tells you whether traffic is flowing.
    const quint32 bps = m.ethBytesPerSec;
    QString rate;
    if (bps >= 1000000u)
        rate = QString("%1 MB/s").arg(bps / 1000000.0, 0, 'f', 2);
    else if (bps >= 1000u)
        rate = QString("%1 kB/s").arg(bps / 1000.0, 0, 'f', 1);
    else
        rate = QString("%1 B/s").arg(bps);

    m_ethText->setText(m.ethLinkMbits > 0
                           ? QString("%1 @ %2M").arg(rate).arg(m.ethLinkMbits)
                           : rate);

    // Presence flags, in kSensors[] order.
    const bool present[SensorCount] = { m.imuPresent, m.magPresent,
                                        m.baroPresent, m.gnssPresent };
    for (int i = 0; i < SensorCount; ++i)
        setPresence(i, present[i] ? Presence::Present : Presence::Absent);

    // ESC telemetry-alive, decoded generically against whichever block
    // Xcp_Esc landed in -- same A2lModel::decodeFrom() the Sensors tab and
    // AttitudeView use, so nothing here assumes a fixed address. An A2L that
    // does not describe EscM<n>Alive (older firmware) leaves that index at -1
    // from setAvailable() and the lamp simply stays at "unknown".
    for (int i = 0; i < ActuatorCount; ++i) {
        Presence p = Presence::Unknown;
        double v = 0.0;
        if (m_idxEscAlive[i] >= 0 &&
            A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[m_idxEscAlive[i]], &v)) {
            p = (v != 0.0) ? Presence::Present : Presence::Absent;
        }
        setActuatorPresence(i, p);
    }

    if (m_idxEscState >= 0) {
        double v = 0.0;
        if (A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[m_idxEscState], &v))
            m_actuatorStateVal->setText(escMotorStateText(int(v)));
        else
            m_actuatorStateVal->setText("--");
    } else {
        m_actuatorStateVal->setText("--");
    }
}
