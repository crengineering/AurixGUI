#include "systemfooter.h"

#include <QLabel>
#include <QProgressBar>
#include <QHBoxLayout>
#include <QFrame>

namespace {
/* Windows before a non-incrementing alive counter is called a hung core. The
 * firmware bumps it every 100 ms and the GUI polls slower, so a couple of
 * repeats are normal; three in a row are not. */
constexpr int STALE_LIMIT = 3;

QFrame *makeSeparator()
{
    auto *line = new QFrame;
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Sunken);
    return line;
}
} // namespace

SystemFooter::SystemFooter(QWidget *parent)
    : QWidget(parent)
{
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

    row->addStretch(1);
    setConnected(false);
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

        m_coreValue[i]->setStyleSheet(m_staleCount[i] >= STALE_LIMIT
                                          ? "color: red; font-weight: bold;"
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
}
