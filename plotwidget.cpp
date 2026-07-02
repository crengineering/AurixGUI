#include "plotwidget.h"

#include <QPainter>
#include <QPaintEvent>
#include <cmath>

PlotWidget::PlotWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumHeight(200);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::white);
    setPalette(pal);
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

void PlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const QRectF area = QRectF(rect()).adjusted(50, 10, -10, -25);
    p.setPen(Qt::gray);
    p.drawRect(area);

    const double tMax = m_tLatest;
    const double tMin = tMax - WINDOW_S;

    // y autoscale over the visible points of visible series
    double yMin = 0.0, yMax = 1.0;
    bool   any = false;
    for (const Series &s : m_series) {
        if (!s.visible)
            continue;
        for (const QPointF &pt : s.points) {
            if (pt.x() < tMin)
                continue;
            if (!any) { yMin = yMax = pt.y(); any = true; }
            yMin = qMin(yMin, pt.y());
            yMax = qMax(yMax, pt.y());
        }
    }
    if (!any) {
        p.drawText(area, Qt::AlignCenter, "keine Daten");
        return;
    }
    if (yMax - yMin < 1e-6) { yMax += 0.5; yMin -= 0.5; }
    const double yPad = (yMax - yMin) * 0.08;
    yMin -= yPad;
    yMax += yPad;

    auto mapX = [&](double t) {
        return area.left() + (t - tMin) / WINDOW_S * area.width();
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
        const double t = tMin + WINDOW_S * i / 6.0;
        const double x = mapX(t);
        p.setPen(QColor(230, 230, 230));
        p.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()));
        p.setPen(Qt::black);
        p.drawText(QRectF(x - 30, area.bottom() + 4, 60, 16), Qt::AlignCenter,
                   QString::number(t - tMax, 'f', 0) + " s");
    }

    // series polylines
    p.setClipRect(area);
    for (const Series &s : m_series) {
        if (!s.visible || s.points.isEmpty())
            continue;
        p.setPen(QPen(s.color, 1.5));
        QPolygonF poly;
        for (const QPointF &pt : s.points) {
            if (pt.x() < tMin - 1.0)
                continue;
            poly << QPointF(mapX(pt.x()), mapY(pt.y()));
        }
        p.drawPolyline(poly);
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
