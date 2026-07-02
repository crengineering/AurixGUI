#ifndef XCPPANEL_H
#define XCPPANEL_H

#include <QWidget>
#include <QElapsedTimer>
#include "xcpclient.h"
#include "mf4writer.h"

class QLineEdit;
class QSpinBox;
class QPushButton;
class QLabel;
class QPlainTextEdit;
class QTableWidget;
class QTabWidget;
class QCheckBox;
class QTimer;
class PlotWidget;

// "Ethernet" tab: connects to the XCP slave on the TC399. Contains
// three sub-tabs: live measurements, diagnostics interpretation (bitmask
// per DIAGNOSTICS.md) and the calibration editor for the threshold block.
class XcpPanel : public QWidget
{
    Q_OBJECT

public:
    explicit XcpPanel(QWidget *parent = nullptr);

signals:
    void connectionChanged(bool connected);   // drives the tab lamp icon

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
    void onDaqStarted();
    void onDaqFailed();
    void startLogging();
    void stopLogging();
    void saveLogging();

private:
    QWidget *buildLiveTab();
    QWidget *buildDiagTab();
    QWidget *buildCalTab();
    QWidget *buildPlotTab();
    void     setConnectedState(bool connected);
    void     updateDiagTable(quint32 status);
    void     updateLogStatus();

    void     updateDiagLamp();

    XcpClient      *m_client     = nullptr;
    QTimer         *m_pollTimer  = nullptr;
    QTabWidget     *m_subTabs    = nullptr;   // Diagnose tab carries a lamp icon

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
    QLabel         *m_diagLamp    = nullptr;  // green = all good, red = error
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

    // plot & logging tab
    PlotWidget     *m_plot        = nullptr;
    QCheckBox      *m_plotChk[5]  = {};
    int             m_series[5]   = {};
    QPushButton    *m_logStartBtn = nullptr;
    QPushButton    *m_logStopBtn  = nullptr;
    QPushButton    *m_logSaveBtn  = nullptr;
    QLabel         *m_logStatus   = nullptr;
    Mf4Writer       m_mf4;
    bool            m_logging     = false;
    QElapsedTimer   m_timeBase;         // running since connect (plot x-axis)
    qint64          m_logStartMs  = 0;
    qint64          m_lastDaqMs   = 0;  // DAQ stream watchdog
};

#endif // XCPPANEL_H
