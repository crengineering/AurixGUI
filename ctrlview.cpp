#include "ctrlview.h"
#include "ctrlmath.h"
#include "motormath.h"
#include "plotwidget.h"

#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <QKeyEvent>
#include <QFocusEvent>
#include <QLineEdit>
#include <QSignalBlocker>
#include <QFont>
#include <cmath>
#include <functional>

namespace {

constexpr int kStaleMs    = 1500;   // DAQ runs at 100 ms; >1.5 s without a sample = stale
constexpr int kCalStaleMs = 3000;   // CAL_ctrl* read-back polled at 2 Hz; >3 s old = unknown
constexpr int kConfirmMs  = 1500;   // an unconfirmed write is given up after this
constexpr double kPlotWindowS = 10.0;

const QColor kColLive(46, 158, 62);
const QColor kColGrey(127, 140, 141);
const QColor kColAmber(185, 119, 14);
const QColor kColRed(192, 57, 43);

int indexOfMeas(const QVector<A2lMeas> &meas, const QString &name)
{
    for (int i = 0; i < meas.size(); ++i)
        if (meas[i].name == name)
            return i;
    return -1;
}

const A2lChar *findChar(const QVector<A2lChar> &chars, const char *name)
{
    for (const A2lChar &c : chars)
        if (c.name == QLatin1String(name))
            return &c;
    return nullptr;
}

QString css(const QColor &c, bool bold = false)
{
    return QStringLiteral("QLabel { color: %1;%2 }").arg(c.name(), bold ? QStringLiteral(" font-weight: bold;") : QString());
}

// Entry field: commits ONLY on Enter/Return (then calls onEnter), never on
// focus-out or the wheel; same behaviour as the Motors tab's entry.
class EntrySpin : public QDoubleSpinBox
{
public:
    using QDoubleSpinBox::QDoubleSpinBox;
    std::function<void()> onEnter;
protected:
    void wheelEvent(QWheelEvent *e) override { e->ignore(); }
    void keyPressEvent(QKeyEvent *e) override
    {
        if (e->key() == Qt::Key_PageUp || e->key() == Qt::Key_PageDown) {
            e->accept();
            return;
        }
        QDoubleSpinBox::keyPressEvent(e);
        if ((e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter) && onEnter)
            onEnter();
    }
    void focusOutEvent(QFocusEvent *e) override
    {
        lineEdit()->setText(prefix() + textFromValue(value()) + suffix());   // revert uncommitted text
        QDoubleSpinBox::focusOutEvent(e);
    }
};

} // namespace

CtrlView::CtrlView(QWidget *parent)
    : QWidget(parent)
{
    buildUi();

    for (QTimer *t : { &m_modeTimer, &m_spTimer, &m_thrTimer })
        t->setSingleShot(true);
    connect(&m_modeTimer, &QTimer::timeout, this, [this]() {
        if (m_modeReq < 0)
            return;
        setNotice(m_idxMode >= 0
                      ? tr("CAL_ctrlMode = %1 was not confirmed by read-back within 1.5 s - the mode shown is the CtrlMode measurement.").arg(m_modeReq)
                      : tr("CAL_ctrlMode = %1 was not confirmed by read-back within 1.5 s - the controller mode in force is unknown.").arg(m_modeReq));
        m_modeReq = -1;
        m_cal.remove(QStringLiteral("CAL_ctrlMode"));
        m_prevMode = -1;
        updateInForce();
        updateControls();
        emit readBackRequested();
    });
    connect(&m_spTimer, &QTimer::timeout, this, [this]() {
        if (!m_spPending)
            return;
        setNotice(tr("The attitude setpoint was not confirmed by read-back within 1.5 s - the setpoint in force is unknown."));
        m_spPending = false;
        updateInForce();
        updateControls();
        emit readBackRequested();
    });
    connect(&m_thrTimer, &QTimer::timeout, this, [this]() {
        if (std::isnan(m_thrReq))
            return;
        setNotice(tr("CAL_ctrlThrustN = %1 N was not confirmed by read-back within 1.5 s - the thrust in force is unknown.").arg(m_thrReq, 0, 'f', 2));
        m_thrReq = qQNaN();
        updateInForce();
        updateControls();
        emit readBackRequested();
    });

    m_staleTimer.setInterval(500);
    connect(&m_staleTimer, &QTimer::timeout, this, &CtrlView::checkStale);
    m_staleTimer.start();
    m_plotClock.start();

    setAvailable({});
    setCharsAvailable({});
    showReadoutsStale();
    updateInForce();
    updateControls();
}

// ---------------------------------------------------------------------------
// layout
// ---------------------------------------------------------------------------

void CtrlView::buildUi()
{
    QFont mono("Consolas");
    mono.setStyleHint(QFont::Monospace);
    QFont bigMono = mono;
    bigMono.setPointSize(mono.pointSize() + 1);

    m_banner = new QLabel;
    m_banner->setWordWrap(true);
    m_banner->setStyleSheet("QLabel { color: white; background: #c0392b; font-weight: bold; padding: 4px; }");
    m_banner->setVisible(false);
    m_bindLbl = new QLabel;
    m_bindLbl->setWordWrap(true);
    m_bindLbl->setStyleSheet(css(kColRed, true));
    m_bindLbl->setVisible(false);

    // ---- column 1: controls ----------------------------------------------
    m_stateLbl = new QLabel("Vehicle: --");
    m_stateLbl->setFont(bigMono);

    // setpoint
    m_spBox = new QGroupBox(tr("Attitude setpoint"));
    auto *spGrid = new QGridLayout(m_spBox);
    const char *axisName[3] = { "Roll", "Pitch", "Yaw" };
    for (int i = 0; i < 3; ++i) {
        auto *spin = new EntrySpin;
        spin->setDecimals(1);
        spin->setSingleStep(1.0);
        spin->setSuffix(" deg");
        spin->setKeyboardTracking(false);
        spin->setAlignment(Qt::AlignRight);
        spin->setFont(bigMono);
        spin->setMinimumWidth(110);
        spin->setFocusPolicy(Qt::ClickFocus);
        spin->setStyleSheet("QDoubleSpinBox { border: 1px solid #8a8a8a; border-radius: 3px; padding: 2px 6px; }"
                            "QDoubleSpinBox:focus { border: 2px solid #2e86de; }");
        spin->setToolTip(tr("Type the setpoint, then press Enter or Apply. The field is yours: the firmware "
                            "read-back does not overwrite it while you are editing."));
        spin->onEnter = [this]() { onApply(); };
        connect(spin, &QDoubleSpinBox::valueChanged, this, [this](double) { m_entryDirty = true; });
        m_spin[i] = spin;
        m_inForce[i] = new QLabel("--");
        m_inForce[i]->setFont(mono);
        spGrid->addWidget(new QLabel(tr(axisName[i]) + ":"), i, 0);
        spGrid->addWidget(spin, i, 1);
        spGrid->addWidget(m_inForce[i], i, 2);
    }
    spGrid->setColumnStretch(2, 1);
    m_hold = new QCheckBox(tr("Hold current heading"));
    m_hold->setChecked(true);
    m_hold->setFocusPolicy(Qt::NoFocus);
    m_hold->setToolTip(tr("Yaw setpoint -1: the firmware keeps the heading it has. Untick to enter a heading 0..360 deg."));
    connect(m_hold, &QCheckBox::toggled, this, &CtrlView::onHoldToggled);
    spGrid->addWidget(m_hold, 3, 1, 1, 2);
    m_applyBtn = new QPushButton(tr("Apply"));
    m_applyBtn->setFocusPolicy(Qt::NoFocus);
    m_applyBtn->setToolTip(tr("Write roll / pitch / yaw (CAL_ctrlAttSp_0..2); the 'in force' column is the firmware's read-back."));
    m_levelBtn = new QPushButton(tr("Level"));
    m_levelBtn->setFocusPolicy(Qt::NoFocus);
    m_levelBtn->setToolTip(tr("Roll 0, pitch 0, hold current heading (0 / 0 / -1) in one go."));
    connect(m_applyBtn, &QPushButton::clicked, this, &CtrlView::onApply);
    connect(m_levelBtn, &QPushButton::clicked, this, &CtrlView::onLevel);
    auto *spBtns = new QHBoxLayout;
    spBtns->addWidget(m_applyBtn);
    spBtns->addWidget(m_levelBtn);
    spGrid->addLayout(spBtns, 4, 0, 1, 3);
    m_reasonLbl = new QLabel;
    m_reasonLbl->setWordWrap(true);
    m_reasonLbl->setStyleSheet(css(kColAmber, true));
    spGrid->addWidget(m_reasonLbl, 5, 0, 1, 3);

    // mode
    auto *modeBox = new QGroupBox(tr("Controller"));
    auto *modeCol = new QVBoxLayout(modeBox);
    m_modeLbl = new QLabel("Mode (CAL_ctrlMode): --");
    m_modeLbl->setFont(bigMono);
    m_shadowBtn = new QPushButton(tr("Shadow"));
    m_driveBtn  = new QPushButton(tr("Drive..."));
    for (QPushButton *b : { m_shadowBtn, m_driveBtn }) {
        b->setCheckable(true);
        b->setFocusPolicy(Qt::NoFocus);
        b->setMinimumHeight(30);
    }
    m_shadowBtn->setToolTip(tr("CAL_ctrlMode = 0: the controller computes and is logged, the motors get nothing from it. "
                               "Always available while connected; goes to the front of the write queue."));
    m_driveBtn->setToolTip(tr("CAL_ctrlMode = 1: the controller drives the motors. Asks for confirmation; only accepted "
                              "by the firmware while ARMED."));
    connect(m_shadowBtn, &QPushButton::clicked, this, &CtrlView::onShadowClicked);
    connect(m_driveBtn,  &QPushButton::clicked, this, &CtrlView::onDriveClicked);
    auto *modeBtns = new QHBoxLayout;
    modeBtns->addWidget(m_shadowBtn);
    modeBtns->addWidget(m_driveBtn);
    m_statusLbl = new QLabel("Status: --");
    m_statusLbl->setWordWrap(true);
    m_tripsLbl = new QLabel("Trips since boot: --");
    m_tripsLbl->setFont(mono);
    m_driveReasonLbl = new QLabel;
    m_driveReasonLbl->setWordWrap(true);
    m_driveReasonLbl->setStyleSheet(css(kColAmber, true));
    m_limitLbl = new QLabel;
    m_limitLbl->setFont(mono);
    modeCol->addWidget(m_modeLbl);
    modeCol->addLayout(modeBtns);
    modeCol->addWidget(m_driveReasonLbl);
    modeCol->addWidget(m_statusLbl);
    modeCol->addWidget(m_tripsLbl);
    modeCol->addWidget(m_limitLbl);

    // thrust
    m_thrBox = new QGroupBox(tr("Collective thrust"));
    auto *thrRow = new QGridLayout(m_thrBox);
    m_thrSpin = new EntrySpin;
    m_thrSpin->setDecimals(2);
    m_thrSpin->setSingleStep(0.5);
    m_thrSpin->setSuffix(" N");
    m_thrSpin->setRange(0.0, m_thrHi);
    m_thrSpin->setKeyboardTracking(false);
    m_thrSpin->setAlignment(Qt::AlignRight);
    m_thrSpin->setFont(bigMono);
    m_thrSpin->setMinimumWidth(110);
    m_thrSpin->setFocusPolicy(Qt::ClickFocus);
    m_thrSpin->setStyleSheet("QDoubleSpinBox { border: 1px solid #8a8a8a; border-radius: 3px; padding: 2px 6px; }"
                             "QDoubleSpinBox:focus { border: 2px solid #2e86de; }");
    static_cast<EntrySpin *>(m_thrSpin)->onEnter = [this]() { onSetThrust(); };
    m_thrBtn = new QPushButton(tr("Set"));
    m_thrBtn->setFocusPolicy(Qt::NoFocus);
    m_thrBtn->setToolTip(tr("Write CAL_ctrlThrustN (set by hand, no altitude loop yet; 0 after every reset)."));
    connect(m_thrBtn, &QPushButton::clicked, this, &CtrlView::onSetThrust);
    m_thrInForce = new QLabel("in force: --");
    m_thrInForce->setFont(mono);
    thrRow->addWidget(m_thrSpin, 0, 0);
    thrRow->addWidget(m_thrBtn, 0, 1);
    thrRow->addWidget(m_thrInForce, 1, 0, 1, 2);
    m_thrReasonLbl = new QLabel;
    m_thrReasonLbl->setWordWrap(true);
    m_thrReasonLbl->setStyleSheet(css(kColAmber, true));
    thrRow->addWidget(m_thrReasonLbl, 2, 0, 1, 2);

    auto *col1 = new QVBoxLayout;
    col1->addWidget(m_stateLbl);
    col1->addWidget(m_spBox);
    col1->addWidget(modeBox);
    col1->addWidget(m_thrBox);
    col1->addStretch(1);
    auto *col1w = new QWidget;
    col1w->setLayout(col1);
    col1w->setMinimumWidth(330);
    col1w->setMaximumWidth(380);

    // ---- column 2: numbers -------------------------------------------------
    auto *numBox = new QGroupBox(tr("Setpoint vs actual"));
    auto *num = new QGridLayout(numBox);
    const char *heads[] = { "", "Setpoint", "Actual", "Error", "Rate sp", "Torque", "Integr." };
    const char *units[] = { "", "[deg]", "[deg]", "[deg]", "[rad/s]", "[Nm]", "[Nm]" };
    for (int c = 0; c < 7; ++c) {
        auto *h = new QLabel(QString("%1\n%2").arg(heads[c], units[c]));
        h->setAlignment(Qt::AlignRight);
        h->setStyleSheet("QLabel { font-weight: bold; }");
        num->addWidget(h, 0, c);
    }
    for (int i = 0; i < 3; ++i) {
        AxisUi &a = m_axis[i];
        QLabel **cells[6] = { &a.sp, &a.act, &a.err, &a.omSoll, &a.tau, &a.tauI };
        num->addWidget(new QLabel(tr(axisName[i])), i + 1, 0);
        for (int c = 0; c < 6; ++c) {
            *cells[c] = new QLabel("--");
            (*cells[c])->setFont(bigMono);
            (*cells[c])->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            (*cells[c])->setMinimumWidth(62);
            num->addWidget(*cells[c], i + 1, c + 1);
        }
    }
    auto *motBox = new QGroupBox(tr("Motors: controller request vs measured"));
    auto *mot = new QGridLayout(motBox);
    const char *mheads[] = { "", "Request [rpm]", "Measured [rpm]", "Diff [rpm]" };
    for (int c = 0; c < 4; ++c) {
        auto *h = new QLabel(mheads[c]);
        h->setAlignment(Qt::AlignRight);
        h->setStyleSheet("QLabel { font-weight: bold; }");
        mot->addWidget(h, 0, c);
    }
    for (int i = 0; i < 4; ++i) {
        mot->addWidget(new QLabel(QString("M%1").arg(i + 1)), i + 1, 0);
        QLabel **cells[3] = { &m_mot[i].req, &m_mot[i].meas, &m_mot[i].diff };
        for (int c = 0; c < 3; ++c) {
            *cells[c] = new QLabel("--");
            (*cells[c])->setFont(bigMono);
            (*cells[c])->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            (*cells[c])->setMinimumWidth(80);
            mot->addWidget(*cells[c], i + 1, c + 1);
        }
    }
    motBox->setToolTip(tr("CtrlM1Rpm..CtrlM4Rpm is what the controller asks for; it reaches the motor only while ARMED "
                          "with Drive. Measured = EscMnBidirRpm (bidirectional DShot telemetry)."));
    auto *col2 = new QVBoxLayout;
    col2->addWidget(numBox);
    col2->addWidget(motBox);
    col2->addStretch(1);
    auto *col2w = new QWidget;
    col2w->setLayout(col2);

    // ---- column 3: plots ---------------------------------------------------
    auto *col3 = new QVBoxLayout;
    for (int i = 0; i < 3; ++i) {
        AxisUi &a = m_axis[i];
        a.plot = new PlotWidget;
        a.plot->setMinimumHeight(90);
        a.plot->setWindowSeconds(kPlotWindowS);
        a.serSp  = a.plot->addSeries(tr("setpoint"), QColor(192, 57, 43));
        a.serAct = a.plot->addSeries(tr("actual"), QColor(46, 134, 222));
        auto *g = new QGroupBox(QString("%1 [deg]").arg(tr(axisName[i])));
        auto *gl = new QVBoxLayout(g);
        gl->setContentsMargins(4, 4, 4, 4);
        gl->addWidget(a.plot);
        col3->addWidget(g, 1);
    }
    auto *col3w = new QWidget;
    col3w->setLayout(col3);
    col3w->setMinimumWidth(240);

    auto *row = new QHBoxLayout;
    row->addWidget(col1w);
    row->addWidget(col2w);
    row->addWidget(col3w, 1);

    auto *content = new QWidget;
    auto *contentCol = new QVBoxLayout(content);
    contentCol->addWidget(m_banner);
    contentCol->addWidget(m_bindLbl);
    contentCol->addLayout(row, 1);

    // Scrolls instead of clipping when the window is small.
    auto *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidget(content);
    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(scroll);
}

// ---------------------------------------------------------------------------
// A2L binding
// ---------------------------------------------------------------------------

void CtrlView::setAvailable(const QVector<A2lMeas> &meas)
{
    m_meas = meas;
    m_idxState  = indexOfMeas(meas, "EscMotorState");
    m_idxStatus = indexOfMeas(meas, "CtrlStatus");
    m_idxTrips  = indexOfMeas(meas, "CtrlTrips");
    m_idxMode   = indexOfMeas(meas, "CtrlMode");            // fw >= 1.19.33: CAL_ctrlMode in force, at DAQ rate
    m_idxTripReason = indexOfMeas(meas, "CtrlTripReason");  // fw >= 1.19.33
    const char *att[3] = { "AttRoll", "AttPitch", "AttYaw" };
    for (int i = 0; i < 3; ++i) {
        m_idxAtt[i]  = indexOfMeas(meas, att[i]);
        m_idxSp[i]   = indexOfMeas(meas, QStringLiteral("CtrlAttSp_%1").arg(i));
        m_idxOm[i]   = indexOfMeas(meas, QStringLiteral("CtrlOmSoll_%1").arg(i));
        m_idxTau[i]  = indexOfMeas(meas, QStringLiteral("CtrlTau_%1").arg(i));
        m_idxTauI[i] = indexOfMeas(meas, QStringLiteral("CtrlTauI_%1").arg(i));
    }
    for (int i = 0; i < 4; ++i) {
        m_idxCtrlRpm[i] = indexOfMeas(meas, QStringLiteral("CtrlM%1Rpm").arg(i + 1));
        m_idxRpm[i]     = indexOfMeas(meas, QStringLiteral("EscM%1BidirRpm").arg(i + 1));
        m_idxRpmOk[i]   = indexOfMeas(meas, QStringLiteral("EscM%1BidirOk").arg(i + 1));
    }
    const bool haveCtrl = m_idxStatus >= 0 && m_idxSp[0] >= 0;
    m_bindLbl->setVisible(!haveCtrl);
    if (!haveCtrl)
        m_bindLbl->setText(tr("CtrlStatus / CtrlAttSp_* not found in the loaded A2L - the flight controller is not part "
                              "of this firmware (needs fw 1.19.32 or later). The panel stays inert."));
    m_haveSample = false;
    showReadoutsStale();
    updateControls();
}

void CtrlView::setCharsAvailable(const QVector<A2lChar> &chars)
{
    m_chars = chars;
    m_modeOk = findChar(chars, "CAL_ctrlMode") != nullptr;
    m_spOk   = findChar(chars, "CAL_ctrlAttSp_0") && findChar(chars, "CAL_ctrlAttSp_1") && findChar(chars, "CAL_ctrlAttSp_2");
    const A2lChar *thr = findChar(chars, "CAL_ctrlThrustN");
    m_thrOk = thr != nullptr;
    if (thr && std::isfinite(thr->hi) && thr->hi > 0.0)
        m_thrHi = thr->hi;
    applyRanges();
    updateControls();
}

// Entry ranges come from the A2L limits of the setpoint words: roll / pitch
// symmetric about zero (lo = -90), yaw 0..hi. The firmware tilt limit
// (CAL_ctrlTiltMaxDeg, read back) is checked at Apply with a visible message
// instead of silently clamping what was typed.
void CtrlView::applyRanges()
{
    double tilt = 90.0, yawHi = 360.0;
    if (const A2lChar *c = findChar(m_chars, "CAL_ctrlAttSp_0"))
        if (std::isfinite(c->lo) && c->lo < 0.0)
            tilt = -c->lo;
    if (const A2lChar *c = findChar(m_chars, "CAL_ctrlAttSp_2"))
        if (std::isfinite(c->hi) && c->hi > 0.0)
            yawHi = c->hi;
    m_spin[0]->setRange(-tilt, tilt);
    m_spin[1]->setRange(-tilt, tilt);
    m_spin[2]->setRange(0.0, yawHi);
    m_thrSpin->setRange(0.0, m_thrHi);
}

// ---------------------------------------------------------------------------
// connection / data
// ---------------------------------------------------------------------------

void CtrlView::setConnected(bool connected)
{
    m_connected = connected;
    m_haveSample = false;
    m_state = -1;
    m_prevState = -1;
    m_haveTrips = false;
    m_faultSeen = 0;
    m_prevMode = -1;
    m_manual = -1;
    m_cal.clear();
    m_haveCal = false;
    m_modeReq = -1;
    m_spPending = false;
    m_thrReq = qQNaN();
    m_modeTimer.stop();
    m_spTimer.stop();
    m_thrTimer.stop();
    m_entryDirty = false;
    m_tripClock.invalidate();
    m_notice.clear();
    m_banner->setVisible(false);
    m_yawPrevRaw = qQNaN();
    if (connected) {
        m_plotClock.restart();
        for (AxisUi &a : m_axis)
            a.plot->clearData();
    }
    showReadoutsStale();
    updateInForce();
    updateControls();
}

bool CtrlView::isStale() const
{
    return !m_connected || !m_haveSample || m_sinceSample.elapsed() > kStaleMs;
}

void CtrlView::checkStale()
{
    if (m_connected && m_haveSample && m_sinceSample.elapsed() > kStaleMs) {
        // Nothing is reset: we do not know what the firmware did. Numbers go
        // grey and the entry is disabled; Shadow stays available.
        showReadoutsStale();
    }
    updateInForce();   // the read-back ages too
    updateControls();
}

void CtrlView::showReadoutsStale()
{
    for (AxisUi &a : m_axis)
        for (QLabel *l : { a.sp, a.act, a.err, a.omSoll, a.tau, a.tauI }) {
            l->setText("--");
            l->setStyleSheet(css(kColGrey));
        }
    for (MotorUi &m : m_mot)
        for (QLabel *l : { m.req, m.meas, m.diff }) {
            l->setText("--");
            l->setStyleSheet(css(kColGrey));
        }
    m_statusLbl->setText(tr("Status: --"));
    m_statusLbl->setStyleSheet(css(kColGrey));
    m_tripsLbl->setText(tr("Trips since boot: --"));
    m_tripsLbl->setStyleSheet(css(kColGrey));
    if (!m_connected)
        m_stateLbl->setText(tr("Vehicle: not connected"));
    else if (m_haveSample)
        m_stateLbl->setText(tr("Vehicle: STALE (no data for > 1.5 s)"));
    else
        m_stateLbl->setText(tr("Vehicle: waiting for data..."));
    m_stateLbl->setStyleSheet(css(kColGrey, true));
}

void CtrlView::feedSample(const XcpClient::Measurements &m)
{
    if (!m_connected)
        return;

    auto dec = [&](int idx, double *out) {
        return idx >= 0 && idx < m_meas.size()
            && A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[idx], out)
            && std::isfinite(*out);
    };
    auto put = [](QLabel *l, double v, int decimals, bool ok) {
        l->setText(ok ? QString::asprintf("%+8.*f", decimals, v) : QStringLiteral("--"));
        l->setStyleSheet(ok ? QString() : css(kColGrey));
    };

    double sv = 0.0;
    if (m_idxState >= 0) {
        if (!dec(m_idxState, &sv))
            return;   // block not covered yet (first sample)
        m_state = int(sv);
    }
    m_haveSample = true;
    m_sinceSample.restart();

    // The firmware resets mode and setpoints on leaving ARMED: drop whatever
    // the operator typed and ask for the read-back now instead of in 500 ms.
    if (m_prevState == kCtrlArmedState && m_state != kCtrlArmedState) {
        m_entryDirty = false;
        emit readBackRequested();
    }
    m_prevState = m_state;

    const QString stateName = motorStateName(m_state);
    m_stateLbl->setText(tr("Vehicle: %1").arg(stateName));
    m_stateLbl->setStyleSheet(css(armed() ? kColRed : (m_state == 3 ? kColAmber : kColLive), true));

    // ---- status / trips -----------------------------------------------------
    double st = 0.0, tr_ = 0.0;
    const bool haveSt = dec(m_idxStatus, &st);
    const bool haveTr = dec(m_idxTrips, &tr_);
    if (haveSt) {
        const quint32 s = quint32(st);
        m_faultSeen |= s & (kCtrlStNavInvalid | kCtrlStTilt | kCtrlStCoreSilent);
        m_statusLbl->setText(tr("Status: %1").arg(ctrlStatusText(s)));
        m_statusLbl->setStyleSheet((s & (kCtrlStNavInvalid | kCtrlStTilt | kCtrlStCoreSilent)) ? css(kColRed, true)
                                   : (s & kCtrlStDriving) ? css(kColAmber, true) : QString());
    } else {
        m_statusLbl->setText(tr("Status: --"));
        m_statusLbl->setStyleSheet(css(kColGrey));
    }
    // Mode in force at DAQ rate (CtrlMode, fw >= 1.19.33). While our own write is
    // unconfirmed the cal read-back decides, so a sample older than the write
    // cannot undo it.
    double mv = 0.0, rv = 0.0;
    if (dec(m_idxMode, &mv) && m_modeReq < 0)
        noteModeChange(mv != 0.0 ? 1 : 0);
    if (dec(m_idxTripReason, &rv))
        m_tripReason = quint32(rv);
    if (haveTr) {
        noteTrip(haveSt ? quint32(st) : 0u, quint32(tr_));
        m_tripsLbl->setText(tr("Trips since boot: %1").arg(quint32(tr_)));
        m_tripsLbl->setStyleSheet(QString());
    } else {
        m_tripsLbl->setText(tr("Trips since boot: --"));
        m_tripsLbl->setStyleSheet(css(kColGrey));
    }

    // ---- per axis ---------------------------------------------------------
    const double t = double(m_plotClock.elapsed()) / 1000.0;
    for (int i = 0; i < 3; ++i) {
        AxisUi &a = m_axis[i];
        double sp = 0, act = 0, om = 0, tau = 0, taui = 0;
        const bool haveSp  = dec(m_idxSp[i], &sp);
        const bool haveAct = dec(m_idxAtt[i], &act);
        put(a.sp, sp, 2, haveSp);
        put(a.act, act, 2, haveAct);
        double err = 0.0;
        const bool haveErr = haveSp && haveAct;
        if (haveErr)
            err = (i == 2) ? ctrlWrap180(act - sp) : act - sp;   // yaw: short way round
        put(a.err, err, 2, haveErr);
        put(a.omSoll, om, 3, dec(m_idxOm[i], &om));
        put(a.tau, tau, 3, dec(m_idxTau[i], &tau));
        put(a.tauI, taui, 3, dec(m_idxTauI[i], &taui));

        if (!haveSp || !haveAct)
            continue;
        if (i == 2) {
            // Heading wraps; plot both on one continuous branch: the actual
            // is unwrapped sample by sample, the setpoint sits at its short-way
            // offset from it, so the seam never draws a vertical line.
            m_yawUnwrapped = std::isnan(m_yawPrevRaw) ? act : ctrlUnwrapNext(m_yawPrevRaw, m_yawUnwrapped, act);
            m_yawPrevRaw = act;
            a.plot->appendPoint(a.serAct, t, m_yawUnwrapped);
            a.plot->appendPoint(a.serSp,  t, m_yawUnwrapped + ctrlWrap180(sp - act));
        } else {
            a.plot->appendPoint(a.serAct, t, act);
            a.plot->appendPoint(a.serSp,  t, sp);
        }
    }

    // ---- motors -----------------------------------------------------------
    for (int i = 0; i < 4; ++i) {
        double req = 0, rpm = 0, ok = 1;
        const bool haveReq = dec(m_idxCtrlRpm[i], &req);
        const bool rpmOk = m_idxRpmOk[i] < 0 || (dec(m_idxRpmOk[i], &ok) && ok != 0.0);   // no valid telemetry = no number
        const bool haveRpm = rpmOk && dec(m_idxRpm[i], &rpm);
        put(m_mot[i].req, req, 0, haveReq);
        put(m_mot[i].meas, rpm, 0, haveRpm);
        put(m_mot[i].diff, rpm - req, 0, haveReq && haveRpm);
    }

    updateInForce();   // the mode label follows CtrlMode at sample rate
    updateControls();
}

// A rise of CtrlTrips = the firmware forced the controller back to shadow.
// Report it once, with the reason from the fault bits seen since the last report.
void CtrlView::noteTrip(quint32 status, quint32 trips)
{
    Q_UNUSED(status);
    if (!m_haveTrips) {
        m_haveTrips = true;
        m_trips = trips;
        m_faultSeen = 0;
        return;
    }
    if (trips > m_trips) {
        const QString reason = ctrlTripReason(m_idxTripReason >= 0 ? m_tripReason : m_faultSeen);
        setNotice(tr("The firmware dropped the controller to SHADOW (trip %1, %2 since the last one): %3. "
                     "Motors get nothing from it; mode and setpoints were reset.")
                      .arg(trips).arg(trips - m_trips).arg(reason));
        m_tripClock.restart();
        m_faultSeen = 0;
        m_entryDirty = false;
        emit readBackRequested();
    }
    m_trips = trips;   // also follows a reset (board rebooted under a live link)
}

// ---------------------------------------------------------------------------
// firmware read-back
// ---------------------------------------------------------------------------

void CtrlView::onMotorCharsRead(const QMap<QString, quint32> &values, bool fresh)
{
    Q_UNUSED(fresh);
    const auto it = values.constFind(QStringLiteral("CAL_motorManual"));
    if (it == values.constEnd())
        return;
    const int now = (it.value() != 0) ? 1 : 0;
    if (now != m_manual) {
        m_manual = now;
        updateControls();
    }
}

void CtrlView::onCharsRead(const QMap<QString, double> &values, bool fresh)
{
    if (values.isEmpty())
        return;
    const double thrBefore = m_cal.value(QStringLiteral("CAL_ctrlThrustN"), qQNaN());
    for (auto it = values.constBegin(); it != values.constEnd(); ++it)
        m_cal.insert(it.key(), it.value());
    {
        // The firmware resets thrust (and the setpoints) to 0 whenever it leaves ARMED or
        // drops the mode. Not an error, but never silent.
        const QString kT = QStringLiteral("CAL_ctrlThrustN");
        if (values.contains(kT) && std::isnan(m_thrReq) && std::isfinite(thrBefore) && thrBefore > 0.0
            && values.value(kT) == 0.0 && m_notice.isEmpty())
            setNotice(tr("Thrust was reset from %1 N to 0 by the firmware (it is only kept while ARMED and is cleared on every "
                         "drop of the controller mode).").arg(thrBefore, 0, 'f', 2));
    }
    m_calAge.restart();
    m_haveCal = true;

    const QString kMode = QStringLiteral("CAL_ctrlMode");
    if (values.contains(kMode) && std::isfinite(values.value(kMode))) {
        const int newMode = values.value(kMode) != 0.0 ? 1 : 0;
        if (m_modeReq >= 0) {
            if (fresh) {          // a reply to a read issued before our write says nothing about it
                const int req = m_modeReq;
                m_modeReq = -1;
                m_modeTimer.stop();
                if (newMode == req) {
                    m_notice.clear();
                    m_banner->setVisible(false);
                    emit logMessage(QStringLiteral("[Ctrl] controller mode confirmed by read-back: CAL_ctrlMode = %1").arg(req));
                } else {
                    setNotice(tr("Drive was not accepted: the firmware reads CAL_ctrlMode back as %1. It is kept only while "
                                 "ARMED and with the Motors tab's manual throttle OFF.").arg(newMode));
                }
                m_prevMode = newMode;
            }
        } else {
            noteModeChange(newMode);
        }
    }

    // setpoint confirmation: all three words of the same fresh read
    static const char *spNames[3] = { "CAL_ctrlAttSp_0", "CAL_ctrlAttSp_1", "CAL_ctrlAttSp_2" };
    if (m_spPending && fresh && values.contains(QLatin1String(spNames[0]))
        && values.contains(QLatin1String(spNames[1])) && values.contains(QLatin1String(spNames[2]))) {
        m_spPending = false;
        m_spTimer.stop();
        bool same = true;
        for (int i = 0; i < 3; ++i)
            same = same && ctrlSameValue(m_spReq[i], values.value(QLatin1String(spNames[i])));
        if (same) {
            m_notice.clear();
            m_banner->setVisible(false);
            emit logMessage(QStringLiteral("[Ctrl] setpoint confirmed by read-back: %1 / %2 / %3")
                                .arg(m_spReq[0], 0, 'f', 1).arg(m_spReq[1], 0, 'f', 1).arg(m_spReq[2], 0, 'f', 1));
        } else {
            setNotice(tr("Setpoint not in force as typed: the firmware reads back roll %1, pitch %2, yaw %3 deg "
                         "(roll / pitch are limited to the tilt limit; the firmware resets them when it is not ARMED).")
                          .arg(values.value(QLatin1String(spNames[0])), 0, 'f', 1)
                          .arg(values.value(QLatin1String(spNames[1])), 0, 'f', 1)
                          .arg(values.value(QLatin1String(spNames[2])), 0, 'f', 1));
        }
        m_entryDirty = false;
    }

    const QString kThr = QStringLiteral("CAL_ctrlThrustN");
    if (!std::isnan(m_thrReq) && fresh && values.contains(kThr)) {
        const double req = m_thrReq;
        m_thrReq = qQNaN();
        m_thrTimer.stop();
        if (ctrlSameValue(req, values.value(kThr))) {
            m_notice.clear();
            m_banner->setVisible(false);
            emit logMessage(QStringLiteral("[Ctrl] thrust confirmed by read-back: %1 N").arg(req, 0, 'f', 2));
        } else {
            setNotice(tr("Thrust %1 N not in force: the firmware reads back %2 N.")
                          .arg(req, 0, 'f', 2).arg(values.value(kThr), 0, 'f', 2));
        }
    }

    syncEntryFromReadBack();
    updateInForce();
    updateControls();
}

// Mode in force: CtrlMode from the DAQ when the A2L has it (then it is as live as the
// samples), else the 2 Hz read-back of CAL_ctrlMode.
bool CtrlView::modeKnown() const
{
    if (m_idxMode >= 0)
        return m_prevMode >= 0 && !isStale();
    return m_prevMode >= 0 && m_haveCal && m_calAge.elapsed() <= kCalStaleMs;
}

// A mode that went 1 -> 0 on its own is reported once: after a trip the trip banner
// already says why, otherwise the vehicle state does.
void CtrlView::noteModeChange(int newMode)
{
    if (m_prevMode == 1 && newMode == 0 && m_modeReq < 0 && !(m_tripClock.isValid() && m_tripClock.elapsed() < 5000)) {
        setNotice(armed()
                      ? tr("The controller went back to SHADOW without a new trip: check the Motors tab (manual throttle) "
                           "and CtrlStatus.")
                      : tr("The controller went back to SHADOW because the vehicle left ARMED (now %1): the firmware "
                           "resets mode, setpoints and thrust.").arg(motorStateName(m_state)));
        m_entryDirty = false;
    }
    m_prevMode = newMode;
}

// The entry fields follow the firmware (e.g. back to 0 / 0 / hold after a
// disarm) unless the operator is typing or has an unapplied value.
void CtrlView::syncEntryFromReadBack()
{
    if (m_entryDirty || m_spPending)
        return;
    for (int i = 0; i < 3; ++i) {
        const QString n = QStringLiteral("CAL_ctrlAttSp_%1").arg(i);
        if (!m_cal.contains(n) || m_spin[i]->hasFocus())
            continue;
        const double v = m_cal.value(n);
        if (!std::isfinite(v))
            continue;
        QSignalBlocker b(m_spin[i]);
        if (i == 2) {
            const bool heading = ctrlYawIsHeading(v);
            QSignalBlocker bh(m_hold);
            m_hold->setChecked(!heading);
            if (heading)
                m_spin[2]->setValue(v);
        } else {
            m_spin[i]->setValue(qBound(m_spin[i]->minimum(), v, m_spin[i]->maximum()));
        }
    }
    if (m_cal.contains(QStringLiteral("CAL_ctrlThrustN")) && !m_thrSpin->hasFocus() && std::isnan(m_thrReq)) {
        const double v = m_cal.value(QStringLiteral("CAL_ctrlThrustN"));
        if (std::isfinite(v)) {
            QSignalBlocker b(m_thrSpin);
            m_thrSpin->setValue(qBound(m_thrSpin->minimum(), v, m_thrSpin->maximum()));
        }
    }
}

// ---------------------------------------------------------------------------
// UI state
// ---------------------------------------------------------------------------

QString CtrlView::entryDisabledReason() const
{
    if (!m_spOk)
        return tr("CAL_ctrlAttSp_* is not in the loaded A2L (firmware without controller wiring).");
    if (!m_connected)
        return tr("Not connected.");
    if (isStale())
        return tr("No live data (stale) - values are not entered blind.");
    if (!armed())
        return tr("Vehicle is %1: the firmware only keeps setpoints while ARMED (arm in the Motors tab or the footer).")
            .arg(motorStateName(m_state));
    if (!m_haveCal || m_calAge.elapsed() > kCalStaleMs)
        return tr("Waiting for the firmware read-back of CAL_ctrl*...");
    if (m_spPending)
        return tr("Waiting for the read-back of the setpoint...");
    return QString();
}

QString CtrlView::driveDisabledReason() const
{
    if (!m_modeOk)
        return tr("CAL_ctrlMode is not in the loaded A2L.");
    if (!m_connected)
        return tr("Not connected.");
    if (isStale())
        return tr("No live data (stale).");
    if (!armed())
        return tr("Drive needs the vehicle ARMED (now %1); the firmware writes the mode back to 0 otherwise.")
            .arg(motorStateName(m_state));
    if (m_manual == 1)
        return tr("Manual throttle is ON in the Motors tab; it wins over the controller.");
    if (!modeKnown() || !m_haveCal)
        return tr("Waiting for the firmware read-back of CAL_ctrlMode...");
    if (m_modeReq >= 0)
        return tr("Waiting for CAL_ctrlMode = %1 to be confirmed by read-back...").arg(m_modeReq);
    if (m_prevMode == 1)
        return QString();   // already driving: button is just the checked state
    return QString();
}

// Thrust follows the setpoint rule: only kept by the firmware while ARMED.
QString CtrlView::thrustDisabledReason() const
{
    if (!m_thrOk)
        return tr("CAL_ctrlThrustN is not in the loaded A2L.");
    if (!m_connected)
        return tr("Not connected.");
    if (isStale())
        return tr("No live data (stale).");
    if (!armed())
        return tr("Vehicle is %1: the firmware keeps thrust only while ARMED and resets it to 0 otherwise.")
            .arg(motorStateName(m_state));
    if (!m_haveCal || m_calAge.elapsed() > kCalStaleMs)
        return tr("Waiting for the firmware read-back of CAL_ctrl*...");
    if (!std::isnan(m_thrReq))
        return tr("Waiting for the read-back of the thrust...");
    return QString();
}

void CtrlView::updateControls()
{
    const QString spReason = entryDisabledReason();
    const bool entry = spReason.isEmpty();
    for (int i = 0; i < 2; ++i)
        m_spin[i]->setEnabled(entry);
    m_hold->setEnabled(entry);
    m_spin[2]->setEnabled(entry && !m_hold->isChecked());
    m_applyBtn->setEnabled(entry);
    m_levelBtn->setEnabled(entry);
    m_reasonLbl->setText(spReason);
    m_reasonLbl->setVisible(!spReason.isEmpty());

    const bool knownMode = m_modeOk && modeKnown() && m_modeReq < 0;
    m_shadowBtn->setChecked(knownMode && m_prevMode == 0);
    m_driveBtn->setChecked(knownMode && m_prevMode == 1);
    // Shadow is the way out: available whenever there is a link, whatever else is unknown.
    m_shadowBtn->setEnabled(m_modeOk && m_connected && (m_prevMode != 0 || m_modeReq >= 0 || !knownMode));
    const QString driveReason = driveDisabledReason();
    m_driveBtn->setEnabled(driveReason.isEmpty() && m_prevMode == 0);
    const bool showReason = !driveReason.isEmpty() && !(m_prevMode == 1);
    m_driveReasonLbl->setText(driveReason);
    m_driveReasonLbl->setVisible(showReason);

    const QString thrReason = thrustDisabledReason();
    const bool thr = thrReason.isEmpty();
    m_thrSpin->setEnabled(thr);
    m_thrBtn->setEnabled(thr);
    m_thrReasonLbl->setText(thrReason);
    m_thrReasonLbl->setVisible(!thrReason.isEmpty());
}

void CtrlView::updateInForce()
{
    const bool calOk = m_haveCal && m_connected && m_calAge.isValid() && m_calAge.elapsed() <= kCalStaleMs;

    for (int i = 0; i < 3; ++i) {
        const QString n = QStringLiteral("CAL_ctrlAttSp_%1").arg(i);
        if (m_spPending) {
            m_inForce[i]->setText(tr("pending..."));
            m_inForce[i]->setStyleSheet(QStringLiteral("QLabel { font-style: italic; }"));
        } else if (!calOk || !m_cal.contains(n) || !std::isfinite(m_cal.value(n))) {
            m_inForce[i]->setText("--");
            m_inForce[i]->setStyleSheet(css(kColGrey));
        } else {
            const double v = m_cal.value(n);
            m_inForce[i]->setText(i == 2 && !ctrlYawIsHeading(v) ? tr("in force: hold")
                                                                 : tr("in force: %1 deg").arg(v, 0, 'f', 1));
            m_inForce[i]->setStyleSheet(QString());
        }
    }

    if (!std::isnan(m_thrReq)) {
        m_thrInForce->setText(tr("pending..."));
        m_thrInForce->setStyleSheet(QStringLiteral("QLabel { font-style: italic; }"));
    } else if (calOk && m_cal.contains(QStringLiteral("CAL_ctrlThrustN"))) {
        m_thrInForce->setText(tr("in force: %1 N").arg(m_cal.value(QStringLiteral("CAL_ctrlThrustN")), 0, 'f', 2));
        m_thrInForce->setStyleSheet(QString());
    } else {
        m_thrInForce->setText(tr("in force: --"));
        m_thrInForce->setStyleSheet(css(kColGrey));
    }

    if (m_modeReq >= 0) {
        m_modeLbl->setText(tr("Mode (CAL_ctrlMode): pending..."));
        m_modeLbl->setStyleSheet(QStringLiteral("QLabel { font-style: italic; }"));
    } else if (!m_modeOk) {
        m_modeLbl->setText(tr("Mode (CAL_ctrlMode): n/a"));
        m_modeLbl->setStyleSheet(css(kColGrey));
    } else if (!modeKnown()) {
        m_modeLbl->setText(tr("Mode (CAL_ctrlMode): --"));
        m_modeLbl->setStyleSheet(css(kColGrey));
    } else {
        m_modeLbl->setText(m_prevMode == 1 ? tr("Mode (CAL_ctrlMode): DRIVE") : tr("Mode (CAL_ctrlMode): SHADOW"));
        m_modeLbl->setStyleSheet(m_prevMode == 1 ? css(kColRed, true) : QString());
    }

    const double tilt = m_cal.value(QStringLiteral("CAL_ctrlTiltMaxDeg"), qQNaN());
    const double rmin = m_cal.value(QStringLiteral("CAL_ctrlRpmMin"), qQNaN());
    if (calOk && std::isfinite(tilt) && std::isfinite(rmin)) {
        m_limitLbl->setText(tr("tilt limit +-%1 deg, rpm min %2").arg(tilt, 0, 'f', 1).arg(rmin, 0, 'f', 0));
        m_limitLbl->setStyleSheet(QString());
    } else {
        m_limitLbl->setText(tr("tilt limit / rpm min: --"));
        m_limitLbl->setStyleSheet(css(kColGrey));
    }
}

void CtrlView::setNotice(const QString &text)
{
    m_notice = text;
    m_banner->setText(text);
    m_banner->setVisible(!text.isEmpty());
    if (!text.isEmpty())
        emit logMessage(QStringLiteral("[Ctrl] ") + text);
}

// ---------------------------------------------------------------------------
// operator actions
// ---------------------------------------------------------------------------

void CtrlView::writeWord(const QString &name, double value)
{
    emit writeCharRequested(name, value);
}

void CtrlView::onHoldToggled(bool hold)
{
    Q_UNUSED(hold);
    m_entryDirty = true;
    updateControls();
}

void CtrlView::onApply()
{
    if (!entryDisabledReason().isEmpty()) {
        updateControls();
        return;
    }
    double v[3] = { m_spin[0]->value(), m_spin[1]->value(),
                    m_hold->isChecked() ? -1.0 : m_spin[2]->value() };
    const double tilt = m_cal.value(QStringLiteral("CAL_ctrlTiltMaxDeg"), qQNaN());
    const double tiltGate = std::isfinite(tilt) ? tilt : m_spin[0]->maximum();
    const char *names[2] = { "Roll", "Pitch" };
    for (int i = 0; i < 2; ++i) {
        if (!ctrlTiltWritable(v[i], tiltGate)) {
            setNotice(tr("%1 %2 deg exceeds the firmware tilt limit of +-%3 deg - nothing sent.")
                          .arg(tr(names[i])).arg(v[i], 0, 'f', 1).arg(tiltGate, 0, 'f', 1));
            return;
        }
    }
    if (!m_hold->isChecked() && !ctrlYawWritable(v[2])) {
        setNotice(tr("Yaw %1 deg is not a heading (0..360) - nothing sent. Tick 'Hold current heading' to keep the heading.")
                      .arg(v[2], 0, 'f', 1));
        return;
    }
    m_notice.clear();
    m_banner->setVisible(false);
    for (int i = 0; i < 3; ++i)
        m_spReq[i] = v[i];
    m_spPending = true;
    m_spTimer.start(kConfirmMs);
    emit logMessage(QStringLiteral("[Ctrl] setpoint requested: roll %1, pitch %2, yaw %3")
                        .arg(v[0], 0, 'f', 1).arg(v[1], 0, 'f', 1).arg(v[2], 0, 'f', 1));
    for (int i = 0; i < 3; ++i)
        writeWord(QStringLiteral("CAL_ctrlAttSp_%1").arg(i), v[i]);
    updateInForce();
    updateControls();
}

void CtrlView::onLevel()
{
    if (!entryDisabledReason().isEmpty()) {
        updateControls();
        return;
    }
    {
        QSignalBlocker b0(m_spin[0]), b1(m_spin[1]), b2(m_hold);
        m_spin[0]->setValue(0.0);
        m_spin[1]->setValue(0.0);
        m_hold->setChecked(true);
    }
    onApply();
}

void CtrlView::onSetThrust()
{
    if (!thrustDisabledReason().isEmpty()) {
        updateControls();
        return;
    }
    const double v = m_thrSpin->value();
    if (!ctrlThrustWritable(v, m_thrHi)) {
        setNotice(tr("Thrust %1 N is outside 0..%2 N - nothing sent.").arg(v, 0, 'f', 2).arg(m_thrHi, 0, 'f', 1));
        return;
    }
    m_notice.clear();
    m_banner->setVisible(false);
    m_thrReq = v;
    m_thrTimer.start(kConfirmMs);
    emit logMessage(QStringLiteral("[Ctrl] thrust requested: %1 N").arg(v, 0, 'f', 2));
    writeWord(QStringLiteral("CAL_ctrlThrustN"), v);
    updateInForce();
    updateControls();
}

void CtrlView::onShadowClicked()
{
    if (!m_connected || !m_modeOk) {
        updateControls();
        return;
    }
    m_notice.clear();
    m_banner->setVisible(false);
    m_modeReq = 0;
    m_modeTimer.start(kConfirmMs);
    emit logMessage(QStringLiteral("[Ctrl] SHADOW requested (CAL_ctrlMode = 0)"));
    writeWord(QStringLiteral("CAL_ctrlMode"), 0.0);   // XcpPanel: purges queued ctrl writes, priority
    updateInForce();
    updateControls();
}

void CtrlView::onDriveClicked()
{
    if (!driveDisabledReason().isEmpty()) {
        updateControls();
        return;
    }
    const double thrust = m_cal.value(QStringLiteral("CAL_ctrlThrustN"), qQNaN());
    const double tilt   = m_cal.value(QStringLiteral("CAL_ctrlTiltMaxDeg"), qQNaN());
    const double rmin   = m_cal.value(QStringLiteral("CAL_ctrlRpmMin"), qQNaN());
    auto num = [](double v, int d) { return std::isfinite(v) ? QString::number(v, 'f', d) : QStringLiteral("?"); };
    const auto answer = QMessageBox::question(
        this, tr("Drive the motors from the flight controller?"),
        tr("This sets CAL_ctrlMode = 1: the flight controller drives the motors from now on.\n\n"
           "Collective thrust %1 N (0 is the normal start: the motors go to the rpm minimum, then raise the thrust), tilt limit +-%2 deg, rpm minimum %3 rpm per motor.\n\n"
           "Shadow (or DISARM) takes it away again; the firmware also drops it on a trip.\n\nDrive?")
            .arg(num(thrust, 2), num(tilt, 1), num(rmin, 0)),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes || !m_connected || !driveDisabledReason().isEmpty())
        return;
    m_notice.clear();
    m_banner->setVisible(false);
    m_modeReq = 1;
    m_modeTimer.start(kConfirmMs);
    emit logMessage(QStringLiteral("[Ctrl] DRIVE requested (CAL_ctrlMode = 1)"));
    writeWord(QStringLiteral("CAL_ctrlMode"), 1.0);
    updateInForce();
    updateControls();
}
