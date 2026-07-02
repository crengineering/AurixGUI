#ifndef XCPPANEL_H
#define XCPPANEL_H

#include <QWidget>
#include "xcpclient.h"

class QLineEdit;
class QSpinBox;
class QPushButton;
class QLabel;
class QPlainTextEdit;
class QTimer;

// "Ethernet (XCP)" tab: connects to the XCP slave on the TC399 and shows
// software version plus cyclic measurements (uptime, die temperature).
class XcpPanel : public QWidget
{
    Q_OBJECT

public:
    explicit XcpPanel(QWidget *parent = nullptr);

private slots:
    void toggleConnection();
    void onConnected(const QString &ident);
    void onDisconnected();
    void onMeasurements(const XcpClient::Measurements &m);
    void onError(const QString &message);
    void pollTick();

private:
    void setConnectedState(bool connected);

    XcpClient      *m_client     = nullptr;
    QTimer         *m_pollTimer  = nullptr;
    QLineEdit      *m_hostEdit   = nullptr;
    QSpinBox       *m_portBox    = nullptr;
    QPushButton    *m_connectBtn = nullptr;
    QLabel         *m_identLbl   = nullptr;
    QLabel         *m_versionLbl = nullptr;
    QLabel         *m_uptimeLbl  = nullptr;
    QLabel         *m_tempLbl    = nullptr;
    QPlainTextEdit *m_log        = nullptr;
};

#endif // XCPPANEL_H
