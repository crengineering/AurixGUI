#include "systemfooter.h"
#include "appicon.h"
#include "lampicon.h"

#include <QLabel>
#include <QProgressBar>
#include <QHBoxLayout>
#include <QFrame>

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

    row->addStretch(1);
    setConnected(false);
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

void SystemFooter::setConnected(bool connected)
{
    m_connected = connected;
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
}
