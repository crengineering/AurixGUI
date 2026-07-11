#ifndef XCPPANEL_H
#define XCPPANEL_H

#include <QWidget>
#include <QElapsedTimer>
#include <QVector>
#include "xcpclient.h"
#include "mf4writer.h"
#include "a2lmodel.h"

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
    void exportCalibration();
    void importCalibration();
    void loadA2lFile();
    void readNvmParams();
    void saveNvmToFlash();
    void loadNvmDefaults();
    void pollNvmCommand();
    void onDaqStarted();
    void onDaqFailed();
    void startLogging();
    void stopLogging();
    void saveLogging();

private:
    QWidget *buildLiveTab();
    QWidget *buildDiagTab();
    QWidget *buildCalTab();
    QWidget *buildNvmTab();
    QWidget *buildPlotTab();
    void     setConnectedState(bool connected);
    void     updateDiagTable(quint32 status);
    void     updateLogStatus();

    void     updateDiagLamp();

    XcpClient      *m_client     = nullptr;
    QTimer         *m_pollTimer  = nullptr;
    QTabWidget     *m_subTabs    = nullptr;   // Diagnostics tab carries a lamp icon

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

    // calibration tab (RAM working page: individual writes, global read).
    // Rows are built from the A2L CHARACTERISTIC list (NVM block excluded);
    // row index == index into m_chars.
    void            writeCalRow(int row);
    void            loadA2l(const QString &path, bool remember);
    void            rebuildCalTable();
    bool            populateCharsFromRead(quint32 base, const QByteArray &data);
    QString         defaultA2lPath() const;
    QVector<A2lChar> m_chars;
    QString         m_a2lPath;
    QLabel         *m_a2lStatus    = nullptr;
    QPushButton    *m_a2lLoadBtn   = nullptr;
    QTableWidget   *m_calTable     = nullptr;
    QPushButton    *m_calReadBtn   = nullptr;
    QPushButton    *m_calExportBtn = nullptr;
    QPushButton    *m_calImportBtn = nullptr;
    QList<QPushButton *> m_calRowBtns;

    // DFLASH tab (persistent Xcp_Nvm block)
    void            writeNvmRow(int row);
    void            sendNvmCommand(quint32 cmd, const QString &name);
    void            setNvmBusy(bool busy);
    QTableWidget   *m_nvmTable    = nullptr;
    QPushButton    *m_nvmReadBtn  = nullptr;
    QPushButton    *m_nvmSaveBtn  = nullptr;   // SAVE -> persist to DFLASH
    QPushButton    *m_nvmDfltBtn  = nullptr;   // DFLT -> reload defaults
    QLabel         *m_nvmStatus   = nullptr;
    QList<QPushButton *> m_nvmRowBtns;
    QString         m_nvmCmdName;
    int             m_nvmRetries  = 0;

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
