#ifndef SYSTEMFOOTER_H
#define SYSTEMFOOTER_H

#include <QWidget>
#include "xcpclient.h"

class QLabel;
class QProgressBar;

/*
 * Permanent status strip along the bottom of the main window: execution time
 * for all six TriCores and the Ethernet link utilisation.
 *
 * It lives below the tab widget rather than inside a tab, so it stays visible
 * whichever tab is open - the point of a load display is that you see it while
 * doing something else.
 *
 * Per core it shows the busy time measured in the last 100 ms window and that
 * time as a percentage. A core whose alive counter stops incrementing is drawn
 * in red: that is a hung core, which otherwise looks identical to an idle one
 * (both report 0 % load).
 */
class SystemFooter : public QWidget
{
    Q_OBJECT

public:
    explicit SystemFooter(QWidget *parent = nullptr);

public slots:
    void update(const XcpClient::Measurements &m);
    void setConnected(bool connected);

private:
    QLabel       *m_coreName[6] = {};
    QLabel       *m_coreValue[6] = {};
    QProgressBar *m_ethBar      = nullptr;
    QLabel       *m_ethText     = nullptr;

    quint16 m_lastAlive[6]  = {0, 0, 0, 0, 0, 0};
    int     m_staleCount[6] = {0, 0, 0, 0, 0, 0};
    bool    m_connected     = false;
};

#endif // SYSTEMFOOTER_H
