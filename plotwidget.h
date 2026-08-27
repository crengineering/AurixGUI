#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QColor>
#include <QPoint>

// Lightweight rolling live plot (no Qt Charts dependency). Shows the last
// windowSeconds of every visible series with a common y-axis.
//
// Interaction (documented in the widget's tooltip too):
//   - mouse wheel                 zoom both axes, centred on the cursor
//   - mouse wheel + Shift         zoom the X (time) axis only
//   - mouse wheel + Ctrl          zoom the Y axis only
//   - left-drag                   pan
//   - double-click                reset to autoscale / follow the live edge
// Zooming or dragging freezes the time window (stops following the live
// edge) and switches the Y axis from autoscale to a fixed range; double-click
// (or PlotPane's "Reset view" button) puts both back to automatic.
class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlotWidget(QWidget *parent = nullptr);

    int  addSeries(const QString &name, const QColor &color);
    void appendPoint(int series, double t, double value);   // t in seconds
    void setSeriesVisible(int series, bool visible);
    void clearData();
    void clearSeries();      // drop all series (used when the A2L is reloaded)

    // Back to the default view: follow the live edge on X, autoscale on Y.
    void resetView();

    // Fixed Y range instead of autoscale (from the PlotPane "Y axis..." dialog).
    void   setYFixedRange(double lo, double hi);
    void   setYAutoscale();
    bool   yAutoscale() const { return !m_yFixed; }
    // Current (or most recently painted) Y range, for seeding that dialog.
    double yRangeMin() const { return m_yFixed ? m_yFixedMin : m_lastYMin; }
    double yRangeMax() const { return m_yFixed ? m_yFixedMax : m_lastYMax; }

protected:
    void paintEvent(QPaintEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    struct Series {
        QString          name;
        QColor           color;
        bool             visible = true;
        QVector<QPointF> points;      // x = t [s], y = value
    };

    QRectF plotArea() const;

    static constexpr double WINDOW_S   = 60.0;    // default visible time window
    static constexpr int    MAX_POINTS = 12000;   // per series (~20 min @ 10 Hz)
    static constexpr double MIN_SPAN_S = 0.5;     // zoom-in limit on X
    static constexpr double MAX_SPAN_S = 24.0 * 3600.0; // zoom-out limit on X

    QVector<Series> m_series;
    double          m_tLatest = 0.0;

    // X (time) view state.
    double m_xSpanS       = WINDOW_S; // zoom level, in seconds
    bool   m_followLatest = true;     // true: right edge trails m_tLatest
    double m_frozenTMax   = 0.0;      // right edge when not following
    double m_lastTMin     = 0.0;      // most recently painted X range, for
    double m_lastTMax     = 0.0;      // zoom/pan pixel<->data math

    // Y view state.
    bool   m_yFixed    = false;       // true: use m_yFixedMin/Max, not autoscale
    double m_yFixedMin = 0.0;
    double m_yFixedMax = 1.0;
    double m_lastYMin  = 0.0;         // most recently painted range, autoscale
    double m_lastYMax  = 1.0;         // or fixed -- the basis for zoom/pan math

    // Drag-to-pan state.
    bool   m_dragging = false;
    QPoint m_dragLastPos;
};

#endif // PLOTWIDGET_H
