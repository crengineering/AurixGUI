#ifndef PLOTPANE_H
#define PLOTPANE_H

#include <QWidget>
#include <QVector>
#include "a2lmodel.h"

class PlotWidget;
class QLabel;
class QPushButton;

/*
 * One configurable plot: a PlotWidget plus its own set of signals, chosen by
 * the user from the A2L MEASUREMENT list.
 *
 * Plots are created empty and on demand rather than showing every signal at
 * once. With ~56 signals in the A2L a single shared plot is unreadable, and
 * mixing radians with g and Pa on one autoscaled axis makes it worse. Letting
 * the user put a few related signals on each plot is what makes the axis
 * scaling useful.
 *
 * The pane owns its series: reconfiguring rebuilds them, so a signal removed
 * from the selection also disappears from the legend.
 */
class PlotPane : public QWidget
{
    Q_OBJECT

public:
    PlotPane(int index, const QVector<A2lMeas> &available, QWidget *parent = nullptr);

    // Replace the signal list offered by the "Signals..." dialog, e.g. after a
    // different A2L is loaded. Any previously chosen signal that no longer
    // exists is dropped.
    void setAvailable(const QVector<A2lMeas> &available);

    // Feed one sample. raw is the firmware measurement block; sensorPresent
    // lets the caller suppress signals whose sensor is missing, so they are not
    // drawn as a flat zero line.
    // Takes every fetched block, not one buffer: a selected signal may live in
    // any of them, and passing only Xcp_Data is what kept the navigation
    // signals off the plot even after the transport was fetching them.
    void append(double t, const QVector<quint32> &bases,
                const QVector<QByteArray> &raws, bool baroPresent, bool imuPresent);

    void clearData();

    QVector<QString> selectedNames() const;
    void             setSelectedNames(const QVector<QString> &names);

signals:
    void removeRequested(PlotPane *pane);
    void configChanged();          // selection edited -> the panel autosaves

private slots:
    void editSignals();

private:
    void rebuildSeries();

    PlotWidget      *m_plot = nullptr;
    QLabel          *m_title = nullptr;
    QVector<A2lMeas> m_available;   // everything the A2L offers
    QVector<A2lMeas> m_selected;    // what this plot draws
    QVector<int>     m_series;      // series id per selected signal
    int              m_index = 0;
};

#endif // PLOTPANE_H
