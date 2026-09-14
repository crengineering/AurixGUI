#include "positionview.h"

#include <QLabel>
#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPainter>
#include <QFont>
#include <cmath>

namespace {

int indexOfMeas(const QVector<A2lMeas> &meas, const char *name)
{
    for (int i = 0; i < meas.size(); ++i)
        if (meas[i].name == QLatin1String(name))
            return i;
    return -1;
}

// Decode helper mirroring AttitudeView's pattern: false on "block not
// covered yet" or non-finite -- caller keeps whatever it already had.
bool decodeIfFinite(const XcpClient::Measurements &m, const QVector<A2lMeas> &meas, int idx, double *out)
{
    if (idx < 0)
        return false;
    double v = 0.0;
    if (!A2lModel::decodeFrom(m.blockBase, m.blockRaw, meas[idx], &v))
        return false;
    if (!std::isfinite(v))
        return false;
    *out = v;
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// PositionTrailWidget: top-down N/E trail (SYS2-GUI-003 pt.1). Plain QPainter,
// no I/O, fed already-decoded points by PositionView -- nothing here ever
// touches the network or the filesystem.
// ---------------------------------------------------------------------------
class PositionTrailWidget : public QWidget
{
public:
    struct Point { double north; double east; qint64 ageMs; };

    explicit PositionTrailWidget(QWidget *parent = nullptr) : QWidget(parent)
    {
        setMinimumSize(200, 200);
        setToolTip("Top-down N/E trail, north up, east right. Newest point brightest; "
                   "grey means dead reckoning (no GNSS anchor).");
    }

    void setWindow(PosWindow w) { m_window = w; update(); }
    void setBound(bool bound)   { m_bound = bound; update(); }
    void setHaveSample(bool have) { m_haveSample = have; update(); }
    void setAnchored(bool anchored) { m_anchored = anchored; update(); }
    void setLive(bool live)     { m_live = live; update(); }

    void setTrail(const QVector<Point> &points, double currentNorth, double currentEast)
    {
        m_points = points;
        m_currentNorth = currentNorth;
        m_currentEast  = currentEast;
        update();
    }

    static constexpr qint64 kTrailLengthMs = 60000;

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), palette().color(QPalette::Base));

        if (!m_bound) {
            p.setPen(palette().color(QPalette::Mid));
            p.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap,
                       "NavPosNorth/East/Down not in the loaded A2L");
            return;
        }
        if (!m_haveSample) {
            p.setPen(palette().color(QPalette::Mid));
            p.drawText(rect(), Qt::AlignCenter, "No position sample yet");
            return;
        }

        const QRectF area = rect().adjusted(10, 10, -10, -10);
        const double halfPx = qMin(area.width(), area.height()) / 2.0;
        const QPointF centerPx = area.center();

        auto toPx = [&](double north, double east) {
            const QPointF n = posNormalize(north - m_currentNorth, east - m_currentEast, m_window);
            return centerPx + QPointF(n.x() * halfPx, n.y() * halfPx);
        };

        // window border
        p.setPen(QPen(palette().color(QPalette::Mid), 1, Qt::DashLine));
        p.drawRect(QRectF(centerPx.x() - halfPx, centerPx.y() - halfPx, 2 * halfPx, 2 * halfPx));

        // origin marker (tangent-plane origin sits at N=0, E=0 by convention),
        // shown whenever it lies inside the current window (SYS2-GUI-003 pt.1).
        {
            const QPointF o = toPx(0.0, 0.0);
            if (std::abs(o.x() - centerPx.x()) <= halfPx + 0.5 &&
                std::abs(o.y() - centerPx.y()) <= halfPx + 0.5) {
                p.setPen(QPen(palette().color(QPalette::Text), 1));
                p.drawLine(o + QPointF(-6, 0), o + QPointF(6, 0));
                p.drawLine(o + QPointF(0, -6), o + QPointF(0, 6));
            }
        }

        // trail, oldest..newest, age-faded (newest brightest); grey when the
        // horizontal state is dead reckoning, never claimed as anchored.
        const QColor freshColor = m_anchored ? QColor(0x2e, 0x86, 0xde) : QColor(0x95, 0x95, 0x95);
        QPointF prev;
        bool havePrev = false;
        for (const Point &pt : m_points) {
            const QPointF here = toPx(pt.north, pt.east);
            const double ageFrac = qBound(0.0, double(pt.ageMs) / double(kTrailLengthMs), 1.0);
            QColor c = freshColor;
            c.setAlphaF(qBound(0.08, 1.0 - ageFrac, 1.0));
            if (havePrev) {
                p.setPen(QPen(c, 2));
                p.drawLine(prev, here);
            }
            prev = here;
            havePrev = true;
        }

        // current position marker, always full-bright.
        p.setPen(Qt::NoPen);
        p.setBrush(m_anchored ? QColor(0x27, 0xae, 0x60) : QColor(0x7f, 0x8c, 0x8d));
        p.drawEllipse(toPx(m_currentNorth, m_currentEast), 5, 5);

        if (!m_live) {
            p.fillRect(area, QColor(255, 255, 255, 140));
            p.setPen(QColor(0xc0, 0x39, 0x2b));
            QFont f = p.font();
            f.setBold(true);
            p.setFont(f);
            p.drawText(rect(), Qt::AlignCenter, "STALE - last position frozen");
        }
    }

private:
    PosWindow m_window = PosWindow::W5m;
    QVector<Point> m_points;
    double m_currentNorth = 0.0;
    double m_currentEast  = 0.0;
    bool m_anchored   = false;
    bool m_live       = false;
    bool m_bound      = false;
    bool m_haveSample = false;
};

// ---------------------------------------------------------------------------
// AltitudeBarWidget: vertical bar for Up = -NavPosDown (SYS2-GUI-003 pt.2),
// same window scale as the trail, zero fixed at the tangent-plane/vertical
// origin (Up = 0 at boot / first baro anchor).
// ---------------------------------------------------------------------------
class AltitudeBarWidget : public QWidget
{
public:
    explicit AltitudeBarWidget(QWidget *parent = nullptr) : QWidget(parent)
    {
        setMinimumWidth(60);
        setMinimumHeight(200);
        setToolTip("Up = -NavPosDown, same window scale as the trail.");
    }

    void setWindow(PosWindow w) { m_window = w; update(); }
    void setBound(bool bound)   { m_bound = bound; update(); }
    void setHaveSample(bool have) { m_haveSample = have; update(); }
    void setAnchored(bool anchored) { m_anchored = anchored; update(); }
    void setLive(bool live)     { m_live = live; update(); }
    void setUp(double upM) { m_up = upM; update(); }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.fillRect(rect(), palette().color(QPalette::Base));

        if (!m_bound) {
            p.setPen(palette().color(QPalette::Mid));
            p.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap, "n/a");
            return;
        }
        if (!m_haveSample) {
            p.setPen(palette().color(QPalette::Mid));
            p.drawText(rect(), Qt::AlignCenter, "-");
            return;
        }

        const QRectF area = rect().adjusted(6, 10, -6, -10);
        const double half = posWindowMetres(m_window) / 2.0;
        const double pxPerM = area.height() / (2.0 * half);
        const double zeroY = area.center().y();
        const double barX = area.center().x();

        // zero line (vertical origin)
        p.setPen(QPen(palette().color(QPalette::Mid), 1, Qt::DashLine));
        p.drawLine(QPointF(area.left(), zeroY), QPointF(area.right(), zeroY));

        const double yClamped = qBound(area.top(), zeroY - m_up * pxPerM, area.bottom());

        // filled bar from zero to the (clamped) current level
        const QColor fill = m_anchored ? QColor(0x27, 0xae, 0x60) : QColor(0x7f, 0x8c, 0x8d);
        p.setPen(Qt::NoPen);
        p.setBrush(fill);
        p.drawRect(QRectF(barX - 8, qMin(zeroY, yClamped), 16, std::abs(zeroY - yClamped)));

        // marker + clamp arrow if the true value is outside the window
        p.setPen(QPen(palette().color(QPalette::Text), 2));
        p.drawLine(QPointF(area.left(), yClamped), QPointF(area.right(), yClamped));
        const bool clippedTop    = (zeroY - m_up * pxPerM) < area.top();
        const bool clippedBottom = (zeroY - m_up * pxPerM) > area.bottom();
        if (clippedTop || clippedBottom) {
            p.setPen(palette().color(QPalette::Mid));
            p.drawText(QRectF(area.left(), clippedTop ? area.top() - 14 : area.bottom(),
                              area.width(), 14),
                       Qt::AlignCenter, clippedTop ? "^ off-scale" : "v off-scale");
        }

        p.setPen(palette().color(QPalette::Text));
        p.drawText(QRectF(0, 0, width(), 12), Qt::AlignHCenter, QString("+%1 m").arg(half, 0, 'f', 0));
        p.drawText(QRectF(0, height() - 12, width(), 12), Qt::AlignHCenter,
                   QString("-%1 m").arg(half, 0, 'f', 0));

        if (!m_live) {
            p.fillRect(area, QColor(255, 255, 255, 140));
            p.setPen(QColor(0xc0, 0x39, 0x2b));
            p.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap, "STALE");
        }
    }

private:
    PosWindow m_window = PosWindow::W5m;
    double m_up        = 0.0;
    bool m_anchored     = false;
    bool m_live         = false;
    bool m_bound        = false;
    bool m_haveSample   = false;
};

// ---------------------------------------------------------------------------
// PositionView
// ---------------------------------------------------------------------------

PositionView::PositionView(QWidget *parent)
    : QWidget(parent)
{
    m_trail  = new PositionTrailWidget;
    m_altBar = new AltitudeBarWidget;

    m_bindStatusLbl = new QLabel;
    m_bindStatusLbl->setWordWrap(true);
    m_bindStatusLbl->setStyleSheet("QLabel { color: #c0392b; font-weight: bold; }");
    m_bindStatusLbl->setVisible(false);

    m_windowBox = new QComboBox;
    m_windowBox->addItem("2 m",  int(PosWindow::W2m));
    m_windowBox->addItem("5 m",  int(PosWindow::W5m));
    m_windowBox->addItem("20 m", int(PosWindow::W20m));
    m_windowBox->setCurrentIndex(1);   // design default: 5 m
    connect(m_windowBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PositionView::onWindowChanged);

    QFont mono("Consolas");
    mono.setStyleHint(QFont::Monospace);

    m_nLbl    = new QLabel("-"); m_nLbl->setFont(mono);
    m_eLbl    = new QLabel("-"); m_eLbl->setFont(mono);
    m_upLbl   = new QLabel("-"); m_upLbl->setFont(mono);
    m_velLbl  = new QLabel("-"); m_velLbl->setFont(mono);
    m_horizLbl  = new QLabel("-");
    m_vertLbl   = new QLabel("-");
    m_originLbl = new QLabel("-");
    m_stateLbl  = new QLabel("Not connected");

    auto *windowBox = new QGroupBox("Window");
    auto *windowLay = new QHBoxLayout(windowBox);
    windowLay->addWidget(new QLabel("Range:"));
    windowLay->addWidget(m_windowBox, 1);

    auto *readoutBox = new QGroupBox("Position");
    auto *form = new QFormLayout(readoutBox);
    form->addRow("N:",  m_nLbl);
    form->addRow("E:",  m_eLbl);
    form->addRow("Up:", m_upLbl);
    form->addRow("Vel N/E/D:", m_velLbl);
    form->addRow("Horizontal:", m_horizLbl);
    form->addRow("Vertical:",   m_vertLbl);
    form->addRow("Origin:",     m_originLbl);
    form->addRow("State:",      m_stateLbl);

    auto *side = new QVBoxLayout;
    side->addWidget(windowBox);
    side->addWidget(readoutBox);
    side->addStretch(1);

    auto *sideWidget = new QWidget;
    sideWidget->setLayout(side);
    sideWidget->setMaximumWidth(220);

    auto *content = new QHBoxLayout;
    content->addWidget(m_trail, 3);
    content->addWidget(m_altBar, 0);
    content->addWidget(sideWidget, 0);

    auto *outer = new QVBoxLayout(this);
    outer->addWidget(m_bindStatusLbl);
    outer->addWidget(new QLabel(
        "Top-down N/E trail (north up, east right) and altitude, relative to "
        "the tangent-plane origin. Grey trail = dead reckoning, no GNSS anchor."));
    outer->addLayout(content, 1);

    updateBoundState();
}

void PositionView::setAvailable(const QVector<A2lMeas> &meas)
{
    m_meas = meas;
    m_idxPosN = indexOfMeas(meas, "NavPosNorth");
    m_idxPosE = indexOfMeas(meas, "NavPosEast");
    m_idxPosD = indexOfMeas(meas, "NavPosDown");
    m_idxVelN = indexOfMeas(meas, "NavVelNorth");
    m_idxVelE = indexOfMeas(meas, "NavVelEast");
    m_idxVelD = indexOfMeas(meas, "NavVelDown");
    m_idxVertOk      = indexOfMeas(meas, "NavVerticalOk");
    m_idxHorizOk     = indexOfMeas(meas, "NavHorizontalOk");
    m_idxOriginSet   = indexOfMeas(meas, "NavOriginSet");
    m_idxGnssNavOk   = indexOfMeas(meas, "GnssNavOk");
    m_idxGnssNumSats = indexOfMeas(meas, "GnssNumSats");
    m_idxGnssHAcc    = indexOfMeas(meas, "GnssHAccuracy");
    m_idxAttState    = indexOfMeas(meas, "AttState");

    m_bound = m_idxPosN >= 0 && m_idxPosE >= 0 && m_idxPosD >= 0;
    // A (possibly different) A2L may put a different board's data behind the
    // same names -- a trail carried over from the previous file would be a
    // false trace of movement that never happened.
    resetTrail();
    updateBoundState();
}

void PositionView::updateBoundState()
{
    m_bindStatusLbl->setVisible(!m_bound);
    if (!m_bound) {
        m_bindStatusLbl->setText(
            "NavPosNorth/East/Down not found in the loaded A2L - the position "
            "panel stays inert until a matching A2L is loaded.");
        showFrozenReadout();
    }
    if (m_trail)  m_trail->setBound(m_bound);
    if (m_altBar) m_altBar->setBound(m_bound);
}

void PositionView::showFrozenReadout()
{
    m_nLbl->setText("-");
    m_eLbl->setText("-");
    m_upLbl->setText("-");
    m_velLbl->setText("-");
    m_horizLbl->setText("-");
    m_vertLbl->setText("-");
    m_originLbl->setText("-");
    m_stateLbl->setText(m_bound ? "-" : "n/a");
    if (m_trail) {
        m_trail->setHaveSample(false);
        m_trail->setLive(false);
    }
    if (m_altBar) {
        m_altBar->setHaveSample(false);
        m_altBar->setLive(false);
    }
}

void PositionView::resetTrail()
{
    m_trailPts.clear();
    m_clock.invalidate();
}

void PositionView::pruneTrail(qint64 nowMs)
{
    while (!m_trailPts.isEmpty() && (nowMs - m_trailPts.first().tMs) > kTrailLengthMs)
        m_trailPts.removeFirst();
}

void PositionView::setConnected(bool connected)
{
    // false -> true: a fresh session, possibly a different board/location --
    // a trail from a previous session must not masquerade as this one's.
    if (connected && !m_connected)
        resetTrail();

    m_connected = connected;
    if (!connected) {
        showFrozenReadout();
        m_stateLbl->setText("Disconnected - last position frozen");
    }
}

void PositionView::onWindowChanged(int index)
{
    m_window = static_cast<PosWindow>(m_windowBox->itemData(index).toInt());
    if (m_trail)  m_trail->setWindow(m_window);
    if (m_altBar) m_altBar->setWindow(m_window);
}

void PositionView::feedSample(const XcpClient::Measurements &m)
{
    if (!m_bound || !m_connected)
        return;

    double posN = 0.0, posE = 0.0, posD = 0.0;
    const bool ok = decodeIfFinite(m, m_meas, m_idxPosN, &posN)
                 && decodeIfFinite(m, m_meas, m_idxPosE, &posE)
                 && decodeIfFinite(m, m_meas, m_idxPosD, &posD);
    if (!ok)
        return;   // block not covered yet (e.g. very first sample); keep last state

    double velN = 0.0, velE = 0.0, velD = 0.0;
    const bool haveVelN = decodeIfFinite(m, m_meas, m_idxVelN, &velN);
    const bool haveVelE = decodeIfFinite(m, m_meas, m_idxVelE, &velE);
    const bool haveVelD = decodeIfFinite(m, m_meas, m_idxVelD, &velD);

    double vertOkV = 0.0, horizOkV = 0.0, originSetV = 0.0;
    const bool navVerticalOk   = decodeIfFinite(m, m_meas, m_idxVertOk,    &vertOkV)   && vertOkV   != 0.0;
    const bool navHorizontalOk = decodeIfFinite(m, m_meas, m_idxHorizOk,   &horizOkV)  && horizOkV  != 0.0;
    const bool navOriginSet    = decodeIfFinite(m, m_meas, m_idxOriginSet, &originSetV) && originSetV != 0.0;

    double gnssNavOkV = 0.0, numSatsV = -1.0, hAccV = -1.0;
    const bool gnssNavOk = decodeIfFinite(m, m_meas, m_idxGnssNavOk, &gnssNavOkV) && gnssNavOkV != 0.0;
    const bool haveSats  = decodeIfFinite(m, m_meas, m_idxGnssNumSats, &numSatsV);
    const bool haveHAcc  = decodeIfFinite(m, m_meas, m_idxGnssHAcc, &hAccV);

    // Attitude-state gate, same channel/rule AttitudeView uses: without
    // AttState in the A2L, live-with-caveat (never freeze forever silently);
    // with it, only AttState == 2 (running) is live.
    const bool haveState = m_idxAttState >= 0;
    int state = -1;
    if (haveState) {
        double sv = 0.0;
        if (decodeIfFinite(m, m_meas, m_idxAttState, &sv))
            state = int(sv);
    }
    const bool live = haveState ? (state == 2) : true;

    const double up = posUpFromDown(posD);
    const PosAnchorState anchor = posAnchorState(navHorizontalOk, gnssNavOk, navVerticalOk,
                                                  haveSats ? int(numSatsV) : -1,
                                                  haveHAcc ? hAccV : -1.0);

    // Trail: only accumulated while live, same convention as AttitudeView's
    // 3D pose (frozen, not fed, while calibrating/aligning/no-sensor).
    if (live) {
        if (!m_clock.isValid())
            m_clock.start();
        const qint64 now = m_clock.elapsed();
        m_trailPts.append({posN, posE, now});
        pruneTrail(now);
    }

    m_trail->setAnchored(anchor.horizontalAnchored);
    m_trail->setLive(live);
    m_trail->setHaveSample(true);
    {
        QVector<PositionTrailWidget::Point> pts;
        pts.reserve(m_trailPts.size());
        const qint64 now = m_clock.isValid() ? m_clock.elapsed() : 0;
        for (const TrailPoint &t : m_trailPts)
            pts.append({t.north, t.east, now - t.tMs});
        m_trail->setTrail(pts, posN, posE);
    }

    m_altBar->setAnchored(anchor.verticalAnchored);
    m_altBar->setLive(live);
    m_altBar->setHaveSample(true);
    m_altBar->setUp(up);

    // Readouts equal the plotted values at all times while connected
    // (SYS2-GUI-003 acceptance (a)) -- not gated on "live", same as
    // AttitudeView's roll/pitch/yaw labels; only disconnect blanks them.
    m_nLbl->setText(QString::number(posN, 'f', 2) + " m");
    m_eLbl->setText(QString::number(posE, 'f', 2) + " m");
    m_upLbl->setText(QString::number(up, 'f', 2) + " m");
    if (haveVelN && haveVelE && haveVelD) {
        m_velLbl->setText(QString("%1 / %2 / %3 m/s")
                              .arg(velN, 0, 'f', 2).arg(velE, 0, 'f', 2).arg(-velD, 0, 'f', 2));
    } else {
        m_velLbl->setText("-");
    }

    m_horizLbl->setText(anchor.horizontalText);
    m_horizLbl->setStyleSheet(anchor.horizontalAnchored
        ? "QLabel { color: #27ae60; font-weight: bold; }"
        : "QLabel { color: palette(mid); font-style: italic; }");
    m_vertLbl->setText(anchor.verticalText);
    m_vertLbl->setStyleSheet(anchor.verticalAnchored
        ? "QLabel { color: #27ae60; font-weight: bold; }"
        : "QLabel { color: palette(mid); font-style: italic; }");
    m_originLbl->setText(navOriginSet ? "set" : "not set");

    if (!haveState) {
        m_stateLbl->setText("Running (state unknown - AttState not in A2L)");
    } else {
        switch (state) {
        case 0:  m_stateLbl->setText("Calibrating gyro bias - hold the board still"); break;
        case 1:  m_stateLbl->setText("Aligning - waiting for a usable accel vector"); break;
        case 2:  m_stateLbl->setText("Running"); break;
        case 3:  m_stateLbl->setText("No sensor - IMU absent"); break;
        default: m_stateLbl->setText("-"); break;
        }
    }
}
