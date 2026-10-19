#include "positionview.h"

#include <QLabel>
#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPainter>
#include <QFont>
#include <QFontMetricsF>
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
                   "grey means not anchored (no fix, GNSS untrusted, or not yet latched).");
    }

    void setWindow(PosWindow w) { m_window = w; update(); }
    void setBound(bool bound)   { m_bound = bound; update(); }
    void setHaveSample(bool have) { m_haveSample = have; update(); }
    void setAnchored(bool anchored) { m_anchored = anchored; update(); }
    void setLive(bool live)     { m_live = live; update(); }
    // Origin marker only when the firmware has actually latched a
    // tangent-plane origin (NavOriginSet) -- always true indoors otherwise,
    // which would draw a marker the readout itself labels "not set" (review
    // finding MINOR 6).
    void setHaveOrigin(bool have) { m_haveOrigin = have; update(); }

    void setTrail(const QVector<Point> &points, double currentNorth, double currentEast)
    {
        m_points = points;
        m_currentNorth = currentNorth;
        m_currentEast  = currentEast;
        update();
    }

    // "Speed at the drone marker" (SYS2-GUI-003 pt.3, acceptance (g)):
    // ground speed and travel direction at the current-position marker.
    // haveVelocity false (NavVelNorth/East not decodable this sample) draws
    // neither label nor arrow, same "don't show stale/bogus data" rule as
    // everywhere else in this panel.
    void setVelocity(bool haveVelocity, double groundSpeedMs, QPointF arrowDir)
    {
        m_haveVelocity  = haveVelocity;
        m_groundSpeedMs = groundSpeedMs;
        m_arrowDir      = arrowDir;
        update();
    }

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
        const QRectF windowRect(centerPx.x() - halfPx, centerPx.y() - halfPx, 2 * halfPx, 2 * halfPx);

        // Pinned-to-frame points (origin marker, current-position marker,
        // which is offset (0,0) by definition): clamped mapping is correct
        // here, they are meant to sit on the border when out of view.
        auto toPxClamped = [&](double north, double east) {
            const QPointF n = posNormalize(north - m_currentNorth, east - m_currentEast, m_window);
            return centerPx + QPointF(n.x() * halfPx, n.y() * halfPx);
        };
        // Trail polyline points: UNCLAMPED -- may land far outside the
        // widget. Drawn under a clip rect below instead of being snapped to
        // the border, so an out-of-window trail is clipped away, not
        // repainted as a path along the frame that was never flown (review
        // finding MAJOR 4).
        auto toPxUnclamped = [&](double north, double east) {
            const QPointF n = posNormalizeUnclamped(north - m_currentNorth, east - m_currentEast, m_window);
            return centerPx + QPointF(n.x() * halfPx, n.y() * halfPx);
        };

        // window border
        p.setPen(QPen(palette().color(QPalette::Mid), 1, Qt::DashLine));
        p.drawRect(windowRect);

        // origin marker (tangent-plane origin sits at N=0, E=0 by
        // convention), shown only once the firmware has actually latched one
        // (NavOriginSet) and only while it lies inside the current window.
        if (m_haveOrigin) {
            const QPointF o = toPxClamped(0.0, 0.0);
            if (std::abs(o.x() - centerPx.x()) <= halfPx + 0.5 &&
                std::abs(o.y() - centerPx.y()) <= halfPx + 0.5) {
                p.setPen(QPen(palette().color(QPalette::Text), 1));
                p.drawLine(o + QPointF(-6, 0), o + QPointF(6, 0));
                p.drawLine(o + QPointF(0, -6), o + QPointF(0, 6));
            }
        }

        // trail, oldest..newest, age-faded (newest brightest); grey (a
        // palette role, not a fixed hex, so it stays distinguishable from
        // the anchored colour in a dark theme too) whenever the horizontal
        // state is not anchored -- no fix, untrusted, or not yet latched --
        // never claimed as anchored. Clipped to the window rect: points
        // outside it are cut, not snapped onto the border.
        const QColor freshColor = m_anchored ? QColor(0x2e, 0x86, 0xde) : palette().color(QPalette::Mid);
        p.save();
        p.setClipRect(windowRect);
        QPointF prev;
        bool havePrev = false;
        for (const Point &pt : m_points) {
            const QPointF here = toPxUnclamped(pt.north, pt.east);
            QColor c = freshColor;
            c.setAlphaF(posTrailAlpha(pt.ageMs));
            if (havePrev) {
                p.setPen(QPen(c, 2));
                p.drawLine(prev, here);
            }
            prev = here;
            havePrev = true;
        }
        p.restore();

        // current position marker, always full-bright, pinned to the frame
        // if the window is smaller than the drift (it is offset (0,0) from
        // itself, so clamped/unclamped agree here).
        p.setPen(Qt::NoPen);
        p.setBrush(m_anchored ? QColor(0x27, 0xae, 0x60) : palette().color(QPalette::Mid));
        const QPointF markerPx = toPxClamped(m_currentNorth, m_currentEast);
        p.drawEllipse(markerPx, 5, 5);

        // Velocity arrow + speed label at the marker (SYS2-GUI-003 pt.3):
        // same anchoring colour as the trail, freezes with it under the
        // STALE overlay below. Length = 1 s of travel at the current speed,
        // scaled by the window's own px-per-metre -- so it reads "how far in
        // one second" at a glance and shrinks/grows with the window like
        // everything else on this plot, clamped to stay visible (>= 12 px)
        // and to not swamp a small window (<= 60% of the half-window).
        if (m_haveVelocity) {
            const QColor col = m_anchored ? QColor(0x2e, 0x86, 0xde) : palette().color(QPalette::Mid);
            if (m_arrowDir.x() != 0.0 || m_arrowDir.y() != 0.0) {
                constexpr double kArrowSeconds = 1.0;   // design default: 1 s of travel
                // Below a certain speed the 12 px floor dominates and the arrow
                // no longer grows proportionally with speed -- e.g. at the 20 m
                // window (half = 10 m) with a ~500 px-wide trail widget,
                // pxPerM ~= 50, so speeds below 12/50 = 0.24 m/s all draw the
                // same 12 px arrow (SWE1-GUI-008 states this plainly; it is a
                // stated design default, not a defect).
                const double pxPerM = halfPx / (posWindowMetres(m_window) / 2.0);
                const double lenPx = qBound(12.0, m_groundSpeedMs * kArrowSeconds * pxPerM, halfPx * 0.6);
                const QPointF tip = markerPx + QPointF(m_arrowDir.x() * lenPx, m_arrowDir.y() * lenPx);
                p.setPen(QPen(col, 2));
                p.drawLine(markerPx, tip);
                const QPointF perp(-m_arrowDir.y(), m_arrowDir.x());
                const QPointF back = tip - QPointF(m_arrowDir.x() * 8.0, m_arrowDir.y() * 8.0);
                p.drawLine(tip, back + perp * 4.0);
                p.drawLine(tip, back - perp * 4.0);
            }
            // Keep the label inside the widget: flip to the other side of
            // the marker whenever the default (right/up) offset would run
            // off that edge (review round 2, note 2 -- a marker pinned to
            // the top-right border used to draw the label partly outside
            // the widget).
            const QString speedText = QString("%1 m/s").arg(m_groundSpeedMs, 0, 'f', 2);
            const QSizeF textSize = QFontMetricsF(p.font()).size(Qt::TextSingleLine, speedText);
            double tx = markerPx.x() + 8.0;
            double ty = markerPx.y() - 8.0;
            if (tx + textSize.width() > rect().right())
                tx = markerPx.x() - 8.0 - textSize.width();
            if (ty - textSize.height() < rect().top())
                ty = markerPx.y() + 8.0 + textSize.height();
            p.setPen(col);
            p.drawText(QPointF(tx, ty), speedText);
        }

        if (!m_live) {
            QColor veil = palette().color(QPalette::Window);
            veil.setAlpha(180);
            p.fillRect(area, veil);
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
    bool m_haveOrigin = false;
    bool m_haveVelocity  = false;
    double m_groundSpeedMs = 0.0;
    QPointF m_arrowDir{0.0, 0.0};
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
    // Vertical rate "next to the altitude value" (SYS2-GUI-003 pt.3,
    // acceptance (g)): Up rate = -NavVelDown, m/s. haveRate false (channel
    // not decodable this sample) shows nothing, same degrade rule as
    // everywhere else.
    void setVerticalRate(bool haveRate, double upRateMs) { m_haveRate = haveRate; m_upRateMs = upRateMs; update(); }

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

        // filled bar from zero to the (clamped) current level -- grey is a
        // palette role, not a fixed hex, so it stays distinguishable from
        // the anchored colour in a dark theme too (review finding MINOR 8).
        const QColor fill = m_anchored ? QColor(0x27, 0xae, 0x60) : palette().color(QPalette::Mid);
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

        // Vertical rate "next to the altitude value" (SYS2-GUI-003 pt.3):
        // placed clear of both the 2px marker line AND the off-scale label
        // (review round 2, note 3 -- the two used to be able to overlap near
        // a window edge). Prefers just below the marker, falls back above it
        // if that would run into the bottom edge/off-scale zone, and is
        // finally clamped so it can never land ON the off-scale label's own
        // rect on whichever side the marker is clipped against.
        if (m_haveRate) {
            constexpr double kGap   = 14.0;   // clear of the 2px marker line + antialiasing
            constexpr double kTextH = 12.0;
            const double topLimit    = clippedTop    ? area.top() + kTextH + 2.0 : area.top();
            const double bottomLimit = clippedBottom ? area.bottom() - kTextH - 2.0 : area.bottom();

            double textY = yClamped + kGap;              // prefer below the marker
            if (textY + kTextH > bottomLimit)
                textY = yClamped - kGap - kTextH;         // fall back to above it
            textY = qBound(topLimit, textY, bottomLimit - kTextH);

            const QString rateText = QString("%1%2 m/s")
                .arg(m_upRateMs >= 0.0 ? "+" : "").arg(m_upRateMs, 0, 'f', 2);
            p.setPen(m_anchored ? QColor(0x27, 0xae, 0x60) : palette().color(QPalette::Mid));
            p.drawText(QRectF(area.left(), textY, area.width(), kTextH), Qt::AlignHCenter, rateText);
        }

        if (!m_live) {
            QColor veil = palette().color(QPalette::Window);
            veil.setAlpha(180);
            p.fillRect(area, veil);
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
    bool m_haveRate     = false;
    double m_upRateMs   = 0.0;
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
    // LOCK badge (SWE1-FW-014 -> SWE1-GUI-007 amendment): NavStationaryLocked
    // = 1 explains why the velocities read exactly zero instead of just
    // showing a suspiciously perfect 0.00. Palette-based colour (not a fixed
    // hex) so it stays legible in a dark theme too, same reasoning as the
    // trail/bar's grey.
    m_lockLbl = new QLabel("-");
    m_lockLbl->setAlignment(Qt::AlignCenter);
    m_lockLbl->setToolTip("NavStationaryLocked: the estimator judged the vehicle stationary "
                           "and pinned the velocities to zero and the position held.");

    auto *windowBox = new QGroupBox("Window");
    auto *windowLay = new QHBoxLayout(windowBox);
    windowLay->addWidget(new QLabel("Range:"));
    windowLay->addWidget(m_windowBox, 1);

    auto *readoutBox = new QGroupBox("Position");
    auto *form = new QFormLayout(readoutBox);
    form->addRow("N:",  m_nLbl);
    form->addRow("E:",  m_eLbl);
    form->addRow("Up:", m_upLbl);
    form->addRow("Vel N/E/Up:", m_velLbl);
    form->addRow("Horizontal:", m_horizLbl);
    form->addRow("Vertical:",   m_vertLbl);
    form->addRow("Origin:",     m_originLbl);
    form->addRow("State:",      m_stateLbl);
    form->addRow("Lock:",       m_lockLbl);

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

    m_introLbl = new QLabel;
    m_introLbl->setWordWrap(true);
    updateIntroText(false);   // no origin yet at construction

    auto *outer = new QVBoxLayout(this);
    outer->addWidget(m_bindStatusLbl);
    outer->addWidget(m_introLbl);
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
    m_idxGnssFixType = indexOfMeas(meas, "GnssFixType");
    m_idxGnssNumSats = indexOfMeas(meas, "GnssNumSats");
    m_idxGnssHAcc    = indexOfMeas(meas, "GnssHAccuracy");
    // Optional (SWE1-GUI-007 amendment, fw >= 1.19.26): an older A2L simply
    // does not have these, and the panel falls back to the pre-amendment
    // two-way anchoring text (posAnchorState's haveGnssTrusted = false path).
    m_idxGnssTrusted      = indexOfMeas(meas, "NavGnssTrusted");
    m_idxStationaryLocked = indexOfMeas(meas, "NavStationaryLocked");
    m_idxAttState    = indexOfMeas(meas, "AttState");

    m_bound = m_idxPosN >= 0 && m_idxPosE >= 0 && m_idxPosD >= 0;
    // A (possibly different) A2L may put a different board's data behind the
    // same names -- a trail carried over from the previous file would be a
    // false trace of movement that never happened.
    resetTrail();
    updateBoundState();
}

void PositionView::updateIntroText(bool haveOrigin)
{
    m_introLbl->setText(haveOrigin
        ? "Top-down N/E trail (north up, east right) and altitude, relative to "
          "the tangent-plane origin (first usable GNSS fix). Grey trail = not "
          "anchored (no fix, GNSS untrusted, or not yet latched) -- held, not "
          "drifting."
        : "Top-down N/E trail (north up, east right) and altitude, relative to "
          "power-on (no GNSS origin latched yet). Grey trail = not anchored "
          "(no fix, GNSS untrusted, or not yet latched) -- held, not drifting.");
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
    m_lockLbl->setText("-");
    m_lockLbl->setStyleSheet(QString());
    // Deliberately does NOT call setHaveSample(false): AttitudeView's
    // pattern (attitudeview.cpp setConnected/setLive) keeps the last pose
    // and only desaturates it -- a disconnect must show the frozen trail
    // and altitude bar under the STALE overlay, not "no sample yet" (review
    // finding MAJOR 3). If no sample ever arrived, m_haveSample is still
    // false from construction, so "No position sample yet" is still shown
    // correctly in that case.
    if (m_trail)
        m_trail->setLive(false);
    if (m_altBar)
        m_altBar->setLive(false);
}

void PositionView::resetTrail()
{
    m_trailPts.clear();
    m_clock.invalidate();
    // The widgets keep their own copy of the trail/current-position (fed by
    // setTrail()) so they can still paint a frozen graphic under the STALE
    // overlay while disconnected (MAJOR 3, round 1). But that means a reset
    // while disconnected -- e.g. loading a different A2L, which is exactly
    // when the old trail is "a different board's data" -- must actively
    // clear them too, or the previous board's path keeps painting under the
    // overlay (review round 2, minor 1). setHaveSample(false) here is
    // correct precisely because this is a genuine reset, unlike the
    // disconnect/not-live freeze in showFrozenReadout(), which must NOT
    // clear it.
    if (m_trail) {
        m_trail->setTrail({}, 0.0, 0.0);
        m_trail->setHaveSample(false);
    }
    if (m_altBar)
        m_altBar->setHaveSample(false);
}

void PositionView::pruneTrail(qint64 nowMs)
{
    while (!m_trailPts.isEmpty() && (nowMs - m_trailPts.first().tMs) > kPosTrailLengthMs)
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
    const bool haveHorizVel = haveVelN && haveVelE;
    // Up rate = -NavVelDown, through the SAME flip site as Up position
    // (posUpFromDown) -- one flip site for every down-axis quantity.
    const double velUp = haveVelD ? posUpFromDown(velD) : 0.0;

    double vertOkV = 0.0, horizOkV = 0.0, originSetV = 0.0;
    const bool navVerticalOk   = decodeIfFinite(m, m_meas, m_idxVertOk,    &vertOkV)   && vertOkV   != 0.0;
    const bool navHorizontalOk = decodeIfFinite(m, m_meas, m_idxHorizOk,   &horizOkV)  && horizOkV  != 0.0;
    const bool navOriginSet    = decodeIfFinite(m, m_meas, m_idxOriginSet, &originSetV) && originSetV != 0.0;

    double gnssNavOkV = 0.0, fixTypeV = -1.0, numSatsV = -1.0, hAccV = -1.0;
    const bool gnssNavOk  = decodeIfFinite(m, m_meas, m_idxGnssNavOk, &gnssNavOkV) && gnssNavOkV != 0.0;
    const bool haveFixType = decodeIfFinite(m, m_meas, m_idxGnssFixType, &fixTypeV);
    const bool haveSats    = decodeIfFinite(m, m_meas, m_idxGnssNumSats, &numSatsV);
    const bool haveHAcc    = decodeIfFinite(m, m_meas, m_idxGnssHAcc, &hAccV);

    // Optional (SWE1-GUI-007 amendment, fw >= 1.19.26): decodeIfFinite
    // returns false outright when the channel is not bound (m_idx.. < 0), so
    // haveGnssTrusted/haveStationaryLocked double as the "channel present in
    // this A2L" flag posAnchorState()/posShowLockBadge() need for the
    // fallback.
    double gnssTrustedV = 0.0, stationaryLockedV = 0.0;
    const bool haveGnssTrusted      = decodeIfFinite(m, m_meas, m_idxGnssTrusted, &gnssTrustedV);
    const bool gnssTrusted          = haveGnssTrusted && gnssTrustedV != 0.0;
    const bool haveStationaryLocked = decodeIfFinite(m, m_meas, m_idxStationaryLocked, &stationaryLockedV);
    // posShowLockBadge (positionmath.h) is the SAME function position_selfcheck
    // exercises -- one definition for "does the badge show", not a second
    // inline copy of the "absent channel never shows a false LOCK" rule.
    const bool showLockBadge = posShowLockBadge(haveStationaryLocked, stationaryLockedV != 0.0);

    // Attitude-state gate (posIsLive, positionmath.h), same channel/rule
    // AttitudeView uses: without AttState in the A2L, live-with-caveat
    // (never freeze forever silently); with it, only AttState == 2
    // (running) is live.
    const bool haveState = m_idxAttState >= 0;
    int state = -1;
    if (haveState) {
        double sv = 0.0;
        if (decodeIfFinite(m, m_meas, m_idxAttState, &sv))
            state = int(sv);
    }
    const bool live = posIsLive(haveState, state);

    const double up = posUpFromDown(posD);
    const PosAnchorState anchor = posAnchorState(navHorizontalOk, gnssNavOk, navVerticalOk,
                                                  haveFixType ? int(fixTypeV) : -1,
                                                  haveSats ? int(numSatsV) : -1,
                                                  haveHAcc ? hAccV : -1.0,
                                                  haveGnssTrusted, gnssTrusted);

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
    m_trail->setHaveOrigin(navOriginSet);
    m_trail->setVelocity(haveHorizVel, haveHorizVel ? posGroundSpeed(velN, velE) : 0.0,
                          haveHorizVel ? posVelocityArrowDir(velN, velE) : QPointF(0.0, 0.0));
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
    m_altBar->setVerticalRate(haveVelD, velUp);

    // Readouts equal the plotted values at all times while connected
    // (SYS2-GUI-003 acceptance (a)) -- not gated on "live", same as
    // AttitudeView's roll/pitch/yaw labels; only disconnect blanks them.
    m_nLbl->setText(QString::number(posN, 'f', 2) + " m");
    m_eLbl->setText(QString::number(posE, 'f', 2) + " m");
    m_upLbl->setText(QString::number(up, 'f', 2) + " m");
    if (haveHorizVel && haveVelD) {
        // velUp (computed above via posUpFromDown, the same flip site as the
        // position's Up row) keeps the "Vel N/E/Up:" row's sign convention
        // consistent with "Up:" above it, not a separately-written negation
        // (review finding MAJOR 2).
        m_velLbl->setText(QString("%1 / %2 / %3 m/s")
                              .arg(velN, 0, 'f', 2).arg(velE, 0, 'f', 2).arg(velUp, 0, 'f', 2));
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
    updateIntroText(navOriginSet);

    // LOCK badge (SWE1-FW-014 -> SWE1-GUI-007 amendment): says outright why
    // the velocities are exactly zero instead of leaving Chris to guess.
    // "n/a" (not "-") when the channel is not in the loaded A2L, same
    // "cannot tell, don't pretend" rule as the GNSS detail fields above.
    if (!haveStationaryLocked) {
        m_lockLbl->setText("n/a");
        m_lockLbl->setStyleSheet("QLabel { color: palette(mid); font-style: italic; }");
    } else if (showLockBadge) {
        m_lockLbl->setText("LOCK");
        m_lockLbl->setStyleSheet("QLabel { background: palette(highlight); "
                                  "color: palette(highlighted-text); font-weight: bold; "
                                  "padding: 1px 6px; border-radius: 3px; }");
    } else {
        m_lockLbl->setText("-");
        m_lockLbl->setStyleSheet(QString());
    }

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
