#ifndef XCPPANEL_H
#define XCPPANEL_H

#include <QWidget>
#include <QElapsedTimer>
#include <QVector>
#include "xcpclient.h"
#include "mf4writer.h"
#include "a2lmodel.h"
#include <QJsonObject>

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
class PlotPane;
class QGridLayout;

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
    void measurementsUpdated(const XcpClient::Measurements &m);  // -> SystemFooter

private slots:
    void toggleConnection();
    void tryAutoConnect();                    // auto-connect / retry after a drop
    void onConnected(const QString &ident);
    void onDisconnected();
    void onMeasurements(const XcpClient::Measurements &m);
    void removePlotPane(PlotPane *pane);
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
    int      measurementBlockSize() const;  // Xcp_Data bytes the A2L describes
    void     updateDiagTable(quint32 status);
    void     updateLogStatus();
    bool     chooseLogChannels(QVector<int> *out);   // pre-log channel picker

    void     updateDiagLamp();

    XcpClient      *m_client     = nullptr;
    QTimer         *m_pollTimer  = nullptr;
    // Reconnect after a flash / power cycle without pressing Connect: every
    // drop schedules one retry, until the board answers again.
    QTimer         *m_reconnectTimer = nullptr;
    bool            m_autoConnect    = true;   // cleared by a manual Disconnect
    bool            m_inConnect      = false;  // guards connectToSlave's own
                                               // disconnected() emission
    bool            m_retryLogged    = false;  // "connecting" logged once per burst
    bool            m_retryErrLogged = false;  // ditto for the timeout message
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
    QLabel         *m_baroPressLbl = nullptr;   // BMP388 pressure
    QLabel         *m_baroTempLbl  = nullptr;   // BMP388 temperature
    QLabel         *m_baroAltLbl   = nullptr;   // pressure altitude
    QLabel         *m_imuAccelLbl  = nullptr;   // MPU-6050 accel x/y/z [g]
    QLabel         *m_imuGyroLbl   = nullptr;   // MPU-6050 gyro x/y/z [deg/s]
    QLabel         *m_imuTempLbl   = nullptr;   // MPU-6050 die temperature
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
    QWidget        *buildSensorsTab();
    void            rebuildDiagTable();
    void            addPlotPane();

    // Plot & Log configuration: which plots exist, which signals each draws,
    // and which channels are selected for logging. Setting a test up by hand
    // takes real time, so the layout is restored automatically on start-up and
    // can also be saved to a named file to switch between test setups.
    QJsonObject     configToJson() const;
    void            applyConfig(const QJsonObject &obj);
    void            saveConfigToFile();
    void            loadConfigFromFile();
    void            autosaveConfig();      // -> QSettings, on every change
    void            restoreAutosavedConfig();
    void            relayoutPlots();
    void            rebuildSensorsTab();
    void            updateSensorValues(const XcpClient::Measurements &m);
    void            rebuildCalTable();
    bool            populateCharsFromRead(quint32 base, const QByteArray &data);
    QString         defaultA2lPath() const;
    QVector<A2lChar> m_chars;
    QString         m_a2lPath;

    // Sensors tab: built from the A2L MEASUREMENT list, grouped by name
    // prefix. m_sensorVal[i] is the value label for m_meas[i], so a new
    // MEASUREMENT in the A2L needs no GUI change at all.
    // Diagnostics rows, built from the A2L BIT_MASK measurements.
    struct DiagRow { int bit; QString text; };
    QVector<DiagRow> m_diagRows;
    int              m_diagTabIndex = -1;

    QVector<A2lMeas> m_meas;
    QVector<QLabel*> m_sensorVal;
    QWidget         *m_sensorsPage   = nullptr;
    QLabel          *m_sensorsStatus = nullptr;
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
    // 15 series: DTS,DTSC,VDD,VDDP3,VEXT,BaroP,BaroT,BaroAlt,
    //            AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,ImuTemp
    // Plot series and log channels are built from the A2L MEASUREMENT list
    // (m_meas), so the set of plottable/loggable signals follows the firmware
    // description. m_signalIdx maps a series/channel index to an entry in
    // m_meas; BIT_MASK diagnostics bits are excluded.
    QVector<int>     m_signalIdx;
    // Plot & Log: a user-built set of plots, two per row. Each PlotPane owns
    // its own signal selection and series; none exist until "Add plot".
    QVector<PlotPane *>  m_plotPanes;
    QWidget             *m_plotGridHost = nullptr;
    QGridLayout         *m_plotGrid     = nullptr;
    QPushButton         *m_addPlotBtn   = nullptr;
    QLabel              *m_plotHint     = nullptr;
    QPushButton    *m_logStartBtn = nullptr;
    QPushButton    *m_logStopBtn  = nullptr;
    QPushButton    *m_logSaveBtn  = nullptr;
    QLabel         *m_logStatus   = nullptr;
    Mf4Writer       m_mf4;
    QVector<int>    m_logSel;            // channel indices selected for logging
    bool            m_logging     = false;
    QElapsedTimer   m_timeBase;         // running since connect (plot x-axis)
    qint64          m_logStartMs  = 0;
    qint64          m_lastDaqMs   = 0;  // DAQ stream watchdog
};

#endif // XCPPANEL_H
