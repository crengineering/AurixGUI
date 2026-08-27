#include "plotwidget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include <cmath>

PlotWidget::PlotWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumHeight(200);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setPalette(pal);
    setMouseTracking(false);
    setCursor(Qt::OpenHandCursor);
    setToolTip(tr("Wheel: zoom both axes | +Shift: zoom X | +Ctrl: zoom Y\n"
                  "Drag: pan | Double-click: reset to autoscale"));
}

int PlotWidget::addSeries(const QString &name, const QColor &color)
{
    Series ser;
    ser.name  = name;
    ser.color = color;
    m_series.append(ser);
    return int(m_series.size()) - 1;
}

void PlotWidget::appendPoint(int series, double t, double value)
{
    if (series < 0 || series >= m_series.size())
        return;

    auto &pts = m_series[series].points;
    pts.append(QPointF(t, value));
    if (pts.size() > MAX_POINTS)
        pts.remove(0, pts.size() - MAX_POINTS);

    m_tLatest = qMax(m_tLatest, t);
    update();
}

void PlotWidget::setSeriesVisible(int series, bool visible)
{
    if (series < 0 || series >= m_series.size())
        return;
    m_series[series].visible = visible;
    update();
}

void PlotWidget::clearData()
{
    for (Series &s : m_series)
        s.points.clear();
    m_tLatest = 0.0;
    update();
}

void PlotWidget::clearSeries()
{
    m_series.clear();
    update();
}

void PlotWidget::resetView()
{
    m_xSpanS       = WINDOW_S;
    m_followLatest = true;
    m_yFixed       = false;
    update();
}

void PlotWidget::setYFixedRange(double lo, double hi)
{
    // Degenerate or swapped input (typos in the dialog) would divide by ~0 in
    // mapY -- clamp to a sane minimum span instead of drawing garbage.
    if (!std::isfinite(lo) || !std::isfinite(hi) || hi <= lo)
        hi = lo + 1.0;
    m_yFixed    = true;
    m_yFixedMin = lo;
    m_yFixedMax = hi;
    update();
}

void PlotWidget::setYAutoscale()
{
    m_yFixed = false;
    update();
}

QRectF PlotWidget::plotArea() const
{
    return QRectF(rect()).adjusted(50, 10, -10, -25);
}

void PlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const QRectF area = plotArea();
    p.setPen(Qt::gray);
    p.drawRect(area);

    const double tMax = m_followLatest ? m_tLatest : m_frozenTMax;
    const double tMin = tMax - m_xSpanS;
    m_lastTMin = tMin;
    m_lastTMax = tMax;

    // y range: autoscale over the visible points of visible series, unless
    // the user fixed it via setYFixedRange().
    double yMin = 0.0, yMax = 1.0;
    bool   any = false;
    for (const Series &s : m_series) {
        if (!s.visible)
            continue;
        for (const QPointF &pt : s.points) {
            if (pt.x() < tMin)
                continue;
            // NaN makes every comparison false, so a NaN sample would
            // otherwise silently widen (or fail to widen) the range and
            // never get excluded by the checks above -- skip it explicitly.
            if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()))
                continue;
            if (!any) { yMin = yMax = pt.y(); any = true; }
            yMin = qMin(yMin, pt.y());
            yMax = qMax(yMax, pt.y());
        }
    }
    if (!any && !m_yFixed) {
        p.drawText(area, Qt::AlignCenter, "keine Daten");
        return;
    }
    if (m_yFixed) {
        yMin = m_yFixedMin;
        yMax = m_yFixedMax;
    } else {
        if (yMax - yMin < 1e-6) { yMax += 0.5; yMin -= 0.5; }
        const double yPad = (yMax - yMin) * 0.08;
        yMin -= yPad;
        yMax += yPad;
    }
    m_lastYMin = yMin;
    m_lastYMax = yMax;

    auto mapX = [&](double t) {
        return area.left() + (t - tMin) / m_xSpanS * area.width();
    };
    auto mapY = [&](double v) {
        return area.bottom() - (v - yMin) / (yMax - yMin) * area.height();
    };

    // horizontal grid + y labels
    p.setFont(QFont(font().family(), 8));
    for (int i = 0; i <= 4; ++i) {
        const double v = yMin + (yMax - yMin) * i / 4.0;
        const double y = mapY(v);
        p.setPen(QColor(230, 230, 230));
        p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
        p.setPen(Qt::black);
        p.drawText(QRectF(0, y - 8, 46, 16), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(v, 'g', 4));
    }
    // x labels (seconds relative to now)
    for (int i = 0; i <= 6; ++i) {
        const double t = tMin + m_xSpanS * i / 6.0;
        const double x = mapX(t);
        p.setPen(QColor(230, 230, 230));
        p.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
        p.setPen(Qt::black);
        p.drawText(QRectF(x - 30, area.bottom() + 4, 60, 16), Qt::AlignCenter,
                   QString::number(t - tMax, 'f', 0) + " s");
    }

    // series polylines. A non-finite sample breaks the line into two
    // segments rather than being plotted (which would paint garbage) or
    // silently bridged (which would draw a fake reading across the gap).
    p.setClipRect(area);
    for (const Series &s : m_series) {
        if (!s.visible || s.points.isEmpty())
            continue;
        p.setPen(QPen(s.color, 1.5));
        QPolygonF poly;
        auto flush = [&]() {
            if (poly.size() > 1)
                p.drawPolyline(poly);
            poly.clear();
        };
        for (const QPointF &pt : s.points) {
            if (pt.x() < tMin - 1.0)
                continue;
            if (!std::isfinite(pt.x()) || !std::isfinite(pt.y())) {
                flush();
                continue;
            }
            poly << QPointF(mapX(pt.x()), mapY(pt.y()));
        }
        flush();
    }
    p.setClipping(false);

    // legend
    double lx = area.left() + 8;
    for (const Series &s : m_series) {
        if (!s.visible)
            continue;
        p.setPen(QPen(s.color, 3));
        p.drawLine(QPointF(lx, area.top() + 10), QPointF(lx + 16, area.top() + 10));
        p.setPen(Qt::black);
        const double w = p.fontMetrics().horizontalAdvance(s.name);
        p.drawText(QPointF(lx + 20, area.top() + 14), s.name);
        lx += 26 + w + 14;
    }
}

void PlotWidget::wheelEvent(QWheelEvent *event)
{
    const QRectF area = plotArea();
    const QPointF pos = event->position();
    if (!area.contains(pos)) {
        event->ignore();
        return;
    }

    // One notch (120 units) per step; fractional for high-resolution wheels.
    const double steps  = event->angleDelta().y() / 120.0;
    if (steps == 0.0) {
        event->ignore();
        return;
    }
    const double factor = std::pow(0.85, steps);   // >1 zooms out, <1 zooms in

    const Qt::KeyboardModifiers mods = event->modifiers();
    const bool xOnly = mods.testFlag(Qt::ShiftModifier) && !mods.testFlag(Qt::ControlModifier);
    const bool yOnly = mods.testFlag(Qt::ControlModifier) && !mods.testFlag(Qt::ShiftModifier);
    const bool zoomX = xOnly || (!xOnly && !yOnly);
    const bool zoomY = yOnly || (!xOnly && !yOnly);

    if (zoomX) {
        const double tCursor = m_lastTMin + (pos.x() - area.left()) / area.width() * m_xSpanS;
        const double frac    = (pos.x() - area.left()) / area.width();
        double newSpan = qBound(MIN_SPAN_S, m_xSpanS * factor, MAX_SPAN_S);
        const double newTMin = tCursor - frac * newSpan;
        m_xSpanS     = newSpan;
        m_frozenTMax = newTMin + newSpan;
        m_followLatest = false;
    }
    if (zoomY) {
        const double range   = qMax(1e-9, m_lastYMax - m_lastYMin);
        const double vCursor = m_lastYMin + (area.bottom() - pos.y()) / area.height() * range;
        const double fracFromBottom = (area.bottom() - pos.y()) / area.height();
        double newRange = qMax(1e-9, range * factor);
        const double newYMin = vCursor - fracFromBottom * newRange;
        setYFixedRange(newYMin, newYMin + newRange);
        return;   // setYFixedRange() already calls update()
    }
    update();
}

void PlotWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && plotArea().contains(event->position())) {
        m_dragging    = true;
        m_dragLastPos = event->position().toPoint();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void PlotWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_dragging) {
        QWidget::mouseMoveEvent(event);
        return;
    }

    const QPoint pos = event->position().toPoint();
    const QPoint delta = pos - m_dragLastPos;
    m_dragLastPos = pos;

    const QRectF area = plotArea();
    if (area.width() > 0.0 && delta.x() != 0) {
        const double dtData = delta.x() / area.width() * m_xSpanS;
        m_frozenTMax   = (m_followLatest ? m_tLatest : m_frozenTMax) - dtData;
        m_followLatest = false;
    }
    if (area.height() > 0.0 && delta.y() != 0) {
        const double range  = qMax(1e-9, m_lastYMax - m_lastYMin);
        const double dyData = delta.y() / area.height() * range;
        setYFixedRange(m_lastYMin + dyData, m_lastYMax + dyData);
        return;   // setYFixedRange() already calls update()
    }
    update();
}

void PlotWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_dragging) {
        m_dragging = false;
        setCursor(Qt::OpenHandCursor);
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void PlotWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && plotArea().contains(event->position())) {
        resetView();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}
