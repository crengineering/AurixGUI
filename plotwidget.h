#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QColor>

// Lightweight rolling live plot (no Qt Charts dependency). Shows the last
// windowSeconds of every visible series with a common autoscaled y-axis.
class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlotWidget(QWidget *parent = nullptr);

    int  addSeries(const QString &name, const QColor &color);
    void appendPoint(int series, double t, double value);   // t in seconds
    void setSeriesVisible(int series, bool visible);
    void clearData();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    struct Series {
        QString          name;
        QColor           color;
        bool             visible = true;
        QVector<QPointF> points;      // x = t [s], y = value
    };

    static constexpr double WINDOW_S   = 60.0;   // visible time window
    static constexpr int    MAX_POINTS = 12000;  // per series (~20 min @ 10 Hz)

    QVector<Series> m_series;
    double          m_tLatest = 0.0;
};

#endif // PLOTWIDGET_H
