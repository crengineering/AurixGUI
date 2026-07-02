#ifndef XCPPANEL_H
#define XCPPANEL_H

#include <QWidget>
#include "xcpclient.h"

class QLineEdit;
class QSpinBox;
class QPushButton;
class QLabel;
class QPlainTextEdit;
class QTableWidget;
class QTimer;

// "Ethernet (XCP)" tab: connects to the XCP slave on the TC399. Contains
// three sub-tabs: live measurements, diagnostics interpretation (bitmask
// per DIAGNOSTICS.md) and the calibration editor for the threshold block.
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
    void onMemoryRead(quint32 address, const QByteArray &data);
    void onMemoryWritten(quint32 address);
    void onError(const QString &message);
    void pollTick();
    void readCalibration();
    void writeCalibration();

private:
    QWidget *buildLiveTab();
    QWidget *buildDiagTab();
    QWidget *buildCalTab();
    void     setConnectedState(bool connected);
    void     updateDiagTable(quint32 status);

    XcpClient      *m_client     = nullptr;
    QTimer         *m_pollTimer  = nullptr;

    // connection row
    QLineEdit      *m_hostEdit   = nullptr;
    QSpinBox       *m_portBox    = nullptr;
    QPushButton    *m_connectBtn = nullptr;

    // live tab
    QLabel         *m_identLbl   = nullptr;
    QLabel         *m_versionLbl = nullptr;
    QLabel         *m_uptimeLbl  = nullptr;
    QLabel         *m_tempLbl    = nullptr;
    QLabel         *m_dtscLbl    = nullptr;
    QLabel         *m_vddLbl     = nullptr;
    QLabel         *m_vddp3Lbl   = nullptr;
    QLabel         *m_vextLbl    = nullptr;
    QPlainTextEdit *m_log        = nullptr;

    // diagnostics tab
    QLabel         *m_diagWordLbl = nullptr;
    QLabel         *m_diagSummary = nullptr;
    QTableWidget   *m_diagTable   = nullptr;
    quint32         m_lastStatus  = 0;
    bool            m_haveStatus  = false;

    // calibration tab
    QTableWidget   *m_calTable    = nullptr;
    QPushButton    *m_calReadBtn  = nullptr;
    QPushButton    *m_calWriteBtn = nullptr;
    int             m_calWritesPending = 0;
};

#endif // XCPPANEL_H
