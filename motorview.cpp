#include "motorview.h"
#include "motormath.h"
#include "lampicon.h"

#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QSlider>
#include <QSpinBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <QPainter>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QFocusEvent>
#include <QLineEdit>
#include <QFont>
#include <cmath>

namespace {

constexpr int kStaleMs = 1500;   // DAQ runs at 100 ms; >1.5 s without a sample = stale

const QColor kColLive(46, 158, 62);
const QColor kColGrey(127, 140, 141);
const QColor kColAmber(224, 160, 0);
const QColor kColRed(192, 57, 43);

int indexOfMeas(const QVector<A2lMeas> &meas, const QString &name)
{
    for (int i = 0; i < meas.size(); ++i)
        if (meas[i].name == name)
            return i;
    return -1;
}

const A2lChar *findChar(const QVector<A2lChar> &chars, const QString &name)
{
    for (const A2lChar &c : chars)
        if (c.name == name)
            return &c;
    return nullptr;
}

QString spName(int motor) { return QStringLiteral("CAL_motorManualSp_M%1").arg(motor + 1); }

// A throttle slider / spinbox must never react to the mouse wheel: scrolling
// the page with the cursor over a slider would otherwise change motor throttle.
class NoWheelSlider : public QSlider
{
public:
    using QSlider::QSlider;
protected:
    void wheelEvent(QWheelEvent *e) override { e->ignore(); }
    // Only the four arrow keys (single step 1). Home/End/PageUp/PageDown would
    // jump the throttle (with "Link all four" on all motors) -- swallowed.
    void keyPressEvent(QKeyEvent *e) override
    {
        switch (e->key()) {
        case Qt::Key_Up: case Qt::Key_Down: case Qt::Key_Left: case Qt::Key_Right:
            QSlider::keyPressEvent(e);
            break;
        default:
            e->accept();
        }
    }
};

class NoWheelSpinBox : public QSpinBox
{
public:
    using QSpinBox::QSpinBox;
protected:
    void wheelEvent(QWheelEvent *e) override { e->ignore(); }
    void keyPressEvent(QKeyEvent *e) override
    {
        if (e->key() == Qt::Key_PageUp || e->key() == Qt::Key_PageDown) {
            e->accept();                 // no big jumps
            return;
        }
        QSpinBox::keyPressEvent(e);
    }
    // A typed value commits ONLY on Enter/Return. On focus-out the text is
    // reverted to the current value first, so the base class has nothing new
    // to interpret and no write is produced.
    void focusOutEvent(QFocusEvent *e) override
    {
        const int v = value();
        const QString t = (v == minimum() && !specialValueText().isEmpty())
                              ? specialValueText()
                              : prefix() + textFromValue(v) + suffix();
        lineEdit()->setText(t);
        QSpinBox::focusOutEvent(e);
    }
};

} // namespace

// Vertical throttle bar: filled part = DShot value actually on the wire
// (EscMnDshotCmd), thin marker = value the slider is commanding.
class MotorBar : public QWidget
{
public:
    explicit MotorBar(QWidget *parent = nullptr) : QWidget(parent)
    {
        setMinimumSize(22, 120);
        setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    }
    void setActual(double frac, bool valid) { m_frac = frac; m_valid = valid; update(); }
    void setMarker(double frac, bool shown) { m_marker = frac; m_markerShown = shown; update(); }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        const QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
        p.setPen(QPen(palette().color(QPalette::Mid), 1));
        p.setBrush(palette().color(QPalette::Base));
        p.drawRect(r);
        if (m_valid) {
            const double h = r.height() * qBound(0.0, m_frac, 1.0);
            p.setPen(Qt::NoPen);
            p.setBrush(kColLive);
            p.drawRect(QRectF(r.left() + 1, r.bottom() - h, r.width() - 2, h));
        }
        if (m_markerShown) {
            const double y = r.bottom() - r.height() * qBound(0.0, m_marker, 1.0);
            p.setPen(QPen(m_valid ? QColor(20, 20, 20) : kColGrey, 2));
            p.drawLine(QPointF(r.left() - 1, y), QPointF(r.right() + 1, y));
        }
    }

private:
    double m_frac = 0.0, m_marker = 0.0;
    bool   m_valid = false, m_markerShown = false;
};

MotorView::MotorView(QWidget *parent)
    : QWidget(parent)
{
    QFont mono("Consolas");
    mono.setStyleHint(QFont::Monospace);
    QFont bigMono = mono;
    bigMono.setPointSize(18);
    bigMono.setBold(true);
    QFont smallMono = mono;
    smallMono.setPointSize(8);

    // ---- top row: state banner, link, buttons --------------------------
    m_stateLbl = new QLabel("--");
    QFont stateFont = mono;
    stateFont.setPointSize(26);
    stateFont.setBold(true);
    m_stateLbl->setFont(stateFont);
    m_stateLbl->setMinimumWidth(190);
    m_stateLbl->setToolTip(tr("EscMotorState: DISARMED / INIT (beeping, ~4 s) / ARMED / FAILSAFE (XCP link lost)."));

    m_linkLamp = new QLabel;
    m_linkLbl  = new QLabel;

    m_armBtn = new QPushButton(tr("ARM"));
    m_armBtn->setMinimumHeight(40);
    m_armBtn->setToolTip(tr("Arm the motors (CAL_motorCmd = 1). Asks for confirmation first."));
    m_disarmBtn = new QPushButton(tr("DISARM"));
    m_disarmBtn->setMinimumHeight(40);
    m_disarmBtn->setToolTip(tr("Disarm immediately (CAL_motorCmd = 0). No confirmation - this is the kill button."));
    m_disarmBtn->setStyleSheet(
        "QPushButton { background: #c0392b; color: white; font-weight: bold;"
        " border: 2px solid #8e2a20; border-radius: 4px; padding: 4px 18px; }"
        "QPushButton:hover:!disabled { background: #d64535; }"
        "QPushButton:disabled { background: #7a4a45; color: #cccccc; border-color: #5a3835; }");
    m_stopAllBtn = new QPushButton(tr("STOP ALL"));
    m_stopAllBtn->setMinimumHeight(40);
    m_stopAllBtn->setToolTip(tr("Write 0 (motor stop) to all four manual setpoints. Stays ARMED."));

    auto *linkBox = new QVBoxLayout;
    auto *linkRow = new QHBoxLayout;
    linkRow->addWidget(m_linkLamp);
    linkRow->addWidget(m_linkLbl, 1);
    linkBox->addLayout(linkRow);

    auto *top = new QHBoxLayout;
    top->addWidget(m_stateLbl);
    top->addLayout(linkBox);
    top->addStretch(1);
    top->addWidget(m_armBtn);
    top->addWidget(m_disarmBtn);
    top->addWidget(m_stopAllBtn);

    // ---- second row: manual toggle, link all four, hint ---------------
    m_manualBtn = new QPushButton(tr("Manual throttle: OFF"));
    m_manualBtn->setMinimumHeight(30);
    m_manualBtn->setToolTip(tr("CAL_motorManual: ON = setpoints come from the sliders instead of the flight controller. "
                               "Only available while ARMED; the firmware clears it on every exit from ARMED."));
    m_linkAll = new QCheckBox(tr("Link all four"));
    m_linkAll->setToolTip(tr("Moving one slider moves all four motors."));
    m_hintLbl = new QLabel;
    m_hintLbl->setWordWrap(true);

    auto *row2 = new QHBoxLayout;
    row2->addWidget(m_manualBtn);
    row2->addWidget(m_linkAll);
    row2->addWidget(m_hintLbl, 1);

    // ---- third row: DShot throttle ceiling (CAL_motorSpMax) ---------------
    m_ceilLbl = new QLabel;
    m_ceilLbl->setFont(mono);
    m_ceilLbl->setMinimumWidth(300);
    m_ceilSpin = new NoWheelSpinBox;
    m_ceilSpin->setRange(kMotorDshotMin, kMotorDshotMax);
    m_ceilSpin->setValue(kMotorDshotMax);
    m_ceilSpin->setKeyboardTracking(false);
    m_ceilSpin->setFont(mono);
    m_ceilSpin->setAlignment(Qt::AlignRight);
    m_ceilSpin->setFocusPolicy(Qt::ClickFocus);
    m_ceilBtn = new QPushButton(tr("Set ceiling"));
    auto *row3 = new QHBoxLayout;
    row3->addWidget(m_ceilLbl);
    row3->addWidget(m_ceilSpin);
    row3->addWidget(m_ceilBtn);
    row3->addStretch(1);

    m_bindLbl = new QLabel;
    m_bindLbl->setWordWrap(true);
    m_bindLbl->setStyleSheet("QLabel { color: #c0392b; font-weight: bold; }");
    m_bindLbl->setVisible(false);

    // ---- motor panels ---------------------------------------------------
    // The physical motor arrangement (which of M1..M4 is front-left, spin
    // direction) is not documented in AurixTricore/docs (DSHOT.md,
    // ESC_AM32.md), so the panels are a plain row M1..M4.
    auto *motorsRow = new QHBoxLayout;
    for (int i = 0; i < kMotors; ++i) {
        MotorUi &u = m_ui[i];
        u.box = new QGroupBox(QStringLiteral("M%1").arg(i + 1));
        u.box->setMinimumWidth(150);

        u.rpmLbl = new QLabel("--");
        u.rpmLbl->setFont(bigMono);
        u.rpmLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        u.rpmLbl->setMinimumWidth(110);
        u.rpmCaption = new QLabel(tr("rpm"));
        u.rpmCaption->setFont(smallMono);
        u.rpmCaption->setAlignment(Qt::AlignRight);

        u.bar = new MotorBar;
        u.slider = new NoWheelSlider(Qt::Vertical);
        u.slider->setRange(kMotorSliderStop, kMotorDshotMax);
        u.slider->setValue(kMotorSliderStop);
        u.slider->setSingleStep(1);
        u.slider->setPageStep(50);
        u.slider->setFocusPolicy(Qt::ClickFocus);
        u.slider->setMinimumHeight(120);
        u.spin = new NoWheelSpinBox;
        u.spin->setRange(kMotorSliderStop, kMotorDshotMax);
        u.spin->setValue(kMotorSliderStop);
        u.spin->setSpecialValueText(tr("STOP"));
        u.spin->setKeyboardTracking(false);   // commit on Enter / focus-out / arrows
        u.spin->setFont(mono);
        u.spin->setAlignment(Qt::AlignRight);
        u.spin->setFocusPolicy(Qt::ClickFocus);

        u.dshotLbl  = new QLabel("--");
        u.erpmLbl   = new QLabel("--");
        u.tempLbl   = new QLabel("--");
        u.countsLbl = new QLabel("--");
        u.kRpmLbl   = new QLabel("--");
        u.kErpmLbl  = new QLabel("--");
        u.tlmLbl    = new QLabel("--");
        for (QLabel *l : { u.dshotLbl, u.erpmLbl, u.tempLbl, u.countsLbl, u.kRpmLbl, u.kErpmLbl, u.tlmLbl }) {
            l->setFont(mono);
            l->setAlignment(Qt::AlignRight);
        }
        u.aliveLamp = new QLabel;
        u.aliveLbl  = new QLabel("--");

        auto *aliveRow = new QHBoxLayout;
        aliveRow->addWidget(u.aliveLamp);
        aliveRow->addWidget(u.aliveLbl, 1);

        auto *barRow = new QHBoxLayout;
        barRow->addWidget(u.bar);
        barRow->addWidget(u.slider);
        barRow->addStretch(1);

        auto *form = new QGridLayout;
        auto section = [&](int row, const QString &title) {
            auto *h = new QLabel(title);
            h->setStyleSheet("QLabel { font-weight: bold; }");
            form->addWidget(h, row, 0, 1, 2);
        };
        form->addWidget(new QLabel(tr("DShot out")), 0, 0); form->addWidget(u.dshotLbl, 0, 1);
        section(1, tr("bidir DShot"));
        form->addWidget(new QLabel(tr("eRPM")),   2, 0); form->addWidget(u.erpmLbl,   2, 1);
        form->addWidget(new QLabel(tr("ok")),     3, 0); form->addWidget(u.countsLbl, 3, 1);
        section(4, tr("KISS tlm"));
        form->addWidget(new QLabel(tr("rpm")),    5, 0); form->addWidget(u.kRpmLbl,   5, 1);
        form->addWidget(new QLabel(tr("eRPM x100")), 6, 0); form->addWidget(u.kErpmLbl, 6, 1);
        form->addWidget(new QLabel(tr("temp")),   7, 0); form->addWidget(u.tempLbl,   7, 1);
        form->addWidget(new QLabel(tr("tlm/miss")), 8, 0); form->addWidget(u.tlmLbl,  8, 1);
        form->setColumnStretch(1, 1);

        auto *col = new QVBoxLayout(u.box);
        col->addWidget(u.rpmLbl);
        col->addWidget(u.rpmCaption);
        col->addLayout(barRow, 1);
        col->addWidget(u.spin);
        col->addLayout(form);
        col->addLayout(aliveRow);

        motorsRow->addWidget(u.box, 1);

        connect(u.slider, &QSlider::valueChanged, this, [this, i](int v) { onSliderChanged(i, v); });
        connect(u.spin, qOverload<int>(&QSpinBox::valueChanged), this, [this, i](int v) { onSpinChanged(i, v); });
    }

    // ---- raw KISS values, collapsed by default --------------------------
    m_rawBox = new QGroupBox(tr("Raw KISS voltage / current / mAh - NOT VALID on the GOKU G55M (floating ADC), shown raw"));
    m_rawBox->setCheckable(true);
    m_rawBox->setChecked(false);
    auto *rawRow = new QHBoxLayout(m_rawBox);
    for (int i = 0; i < kMotors; ++i) {
        m_ui[i].rawLbl = new QLabel("--");
        m_ui[i].rawLbl->setFont(smallMono);
        rawRow->addWidget(m_ui[i].rawLbl, 1);
    }
    connect(m_rawBox, &QGroupBox::toggled, this, [this](bool on) {
        for (int i = 0; i < kMotors; ++i)
            m_ui[i].rawLbl->setVisible(on);
    });
    for (int i = 0; i < kMotors; ++i)
        m_ui[i].rawLbl->setVisible(false);

    auto *content = new QWidget;
    auto *lay = new QVBoxLayout(content);
    lay->addLayout(top);
    lay->addLayout(row2);
    lay->addLayout(row3);
    lay->addWidget(m_bindLbl);
    lay->addLayout(motorsRow, 1);
    lay->addWidget(m_rawBox);

    auto *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidget(content);
    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(scroll);

    // ---- wiring -----------------------------------------------------------
    connect(m_armBtn,     &QPushButton::clicked, this, &MotorView::onArmClicked);
    connect(m_disarmBtn,  &QPushButton::clicked, this, [this]() {
        // No confirmation: kill button.
        onDisarmIssued();
        emit disarmRequested();
    });
    connect(m_stopAllBtn, &QPushButton::clicked, this, &MotorView::onStopAll);
    connect(m_ceilBtn,    &QPushButton::clicked, this, &MotorView::onSetCeiling);
    m_ceilTimer.setSingleShot(true);
    connect(&m_ceilTimer, &QTimer::timeout, this, [this]() {
        if (m_ceilReq < 0)
            return;
        m_notice = tr("CAL_motorSpMax = %1 was not confirmed by read-back within 1.5 s - the ceiling in force is unknown.").arg(m_ceilReq);
        emit logMessage(QStringLiteral("[Motors] ") + m_notice);
        m_ceilReq = -1;
        m_ceiling = -1;
        applyRange();
        updateControls();
        emit readBackRequested();
    });
    connect(m_manualBtn,  &QPushButton::clicked, this, [this]() { onManualClicked(m_manual != Manual::On); });

    m_flushTimer.setSingleShot(true);
    m_flushTimer.setInterval(50);
    connect(&m_flushTimer, &QTimer::timeout, this, &MotorView::flushPending);
    m_staleTimer.setInterval(500);
    connect(&m_staleTimer, &QTimer::timeout, this, &MotorView::checkStale);
    m_staleTimer.start();
    m_pollTimer.setInterval(1000);
    connect(&m_pollTimer, &QTimer::timeout, this, [this]() {
        // 1 Hz while ARMED (manual state) and, with a ceiling in the A2L, in
        // every state so a ceiling changed elsewhere is followed.
        if (m_connected && !isStale() && m_charsOk && (m_state == 2 || m_ceilOk))
            emit readBackRequested();
    });
    m_pollTimer.start();
    m_pendTimer.setSingleShot(true);
    connect(&m_pendTimer, &QTimer::timeout, this, &MotorView::pendingTimeout);
    m_lockTimer.setSingleShot(true);
    m_lockTimer.setInterval(3000);
    connect(&m_lockTimer, &QTimer::timeout, this, [this]() { m_locked = false; updateControls(); updateBanner(); });

    showReadoutsStale();
    updateControls();
    updateBanner();
}

// ---------------------------------------------------------------------------
// A2L binding
// ---------------------------------------------------------------------------

void MotorView::setAvailable(const QVector<A2lMeas> &meas)
{
    m_meas = meas;
    m_idxState = indexOfMeas(meas, QStringLiteral("EscMotorState"));
    for (int i = 0; i < kMotors; ++i) {
        const QString p = QStringLiteral("EscM%1").arg(i + 1);
        MotorIdx &x = m_idx[i];
        x.alive     = indexOfMeas(meas, p + "Alive");
        x.dshot     = indexOfMeas(meas, p + "DshotCmd");
        x.bidirRpm  = indexOfMeas(meas, p + "BidirRpm");
        x.bidirErpm = indexOfMeas(meas, p + "BidirErpm");
        x.bidirOk   = indexOfMeas(meas, p + "BidirOk");
        x.rpm       = indexOfMeas(meas, p + "Rpm");
        x.erpm100   = indexOfMeas(meas, p + "ERpm100");
        x.temp      = indexOfMeas(meas, p + "TempC");
        x.tlmCount  = indexOfMeas(meas, p + "TlmCount");
        x.tlmMissed = indexOfMeas(meas, p + "TlmMissed");
        x.volt      = indexOfMeas(meas, p + "VoltageCv");
        x.cur       = indexOfMeas(meas, p + "CurrentCa");
        x.mah       = indexOfMeas(meas, p + "Mah");
    }
    // A2L reload: only re-resolve the indices. The firmware has NOT zeroed
    // anything, so manual/sliders/state keep their values and the next sample
    // and read-back refresh them (a reset here would show OFF/STOP while the
    // motors keep spinning).
    updateControls();
    updateBanner();
}

void MotorView::setCharsAvailable(const QVector<A2lChar> &chars)
{
    m_cmdOk = findChar(chars, QStringLiteral("CAL_motorCmd")) != nullptr;
    bool ok = m_cmdOk && findChar(chars, QStringLiteral("CAL_motorManual")) != nullptr;
    int spMax = kMotorDshotMax;
    m_ceilOk = findChar(chars, QStringLiteral("CAL_motorSpMax")) != nullptr;
    if (!m_ceilOk) {
        m_ceiling = -1;
        m_ceilReq = -1;
    }
    for (int i = 0; i < kMotors; ++i) {
        const A2lChar *c = findChar(chars, spName(i));
        if (!c) { ok = false; continue; }
        // The A2L limit is authoritative, but never beyond DShot's 2047.
        if (std::isfinite(c->hi) && c->hi >= kMotorDshotMin)
            spMax = qMin(spMax, int(c->hi));
    }
    m_charsOk = ok;
    m_spHi = spMax;
    applyRange();
    updateControls();
    updateBanner();
}

// Slider / spinbox maximum = min(A2L limit of the setpoints, firmware ceiling
// read back). The STOP position (47) and the 48..max encoding are unchanged.
// Without CAL_motorSpMax in the A2L, or before it is read back, the fixed A2L
// range applies (the sliders stay disabled until the ceiling is known).
void MotorView::applyRange()
{
    m_spMax = motorEffectiveMax(m_spHi, m_ceilOk ? m_ceiling : -1);
    bool clamped = false;
    for (int i = 0; i < kMotors; ++i) {
        // setMaximum may clamp the value; that must never look like a user move.
        const bool over = m_ui[i].slider->value() > m_spMax;
        const QSignalBlocker b1(m_ui[i].slider), b2(m_ui[i].spin);
        m_ui[i].slider->setMaximum(m_spMax);
        m_ui[i].spin->setMaximum(m_spMax);
        // The firmware clamp never rewrites CAL_motorManualSp_Mn: while manual is
        // ON, write the clamped position so the firmware holds what is shown
        // (otherwise raising the ceiling later would put the old value on the wire).
        if (over && m_manual == Manual::On && m_connected) {
            m_pending[i] = m_spMax;
            clamped = true;
        }
    }
    if (clamped && !m_flushTimer.isActive())
        m_flushTimer.start();
    // A queued slider position above a lowered ceiling is clamped, never sent.
    for (int &p : m_pending)
        if (p > m_spMax)
            p = m_spMax;
    updateCeilingUi();
}

void MotorView::updateCeilingUi()
{
    if (!m_ceilLbl)
        return;
    const bool usable = m_ceilOk && m_connected && !isStale();
    if (!m_ceilOk) {
        m_ceilLbl->setText(tr("Ceiling (CAL_motorSpMax): n/a"));
        m_ceilLbl->setStyleSheet(QStringLiteral("QLabel { color: %1; }").arg(kColGrey.name()));
    } else if (m_ceilReq >= 0) {
        m_ceilLbl->setText(tr("Ceiling (CAL_motorSpMax): pending..."));
        m_ceilLbl->setStyleSheet(QStringLiteral("QLabel { font-style: italic; }"));
    } else if (m_ceiling < 0 || isStale()) {
        m_ceilLbl->setText(tr("Ceiling (CAL_motorSpMax): --"));
        m_ceilLbl->setStyleSheet(QStringLiteral("QLabel { color: %1; }").arg(kColGrey.name()));
    } else {
        m_ceilLbl->setText(tr("Ceiling (CAL_motorSpMax): %1").arg(m_ceiling, 4));
        m_ceilLbl->setStyleSheet(m_ceiling >= kMotorDshotMax
                                     ? QStringLiteral("QLabel { color: %1; font-weight: bold; }").arg(kColAmber.name())
                                     : QString());
    }
    const QString na = tr("firmware without CAL_motorSpMax");
    m_ceilSpin->setEnabled(usable);
    m_ceilBtn->setEnabled(usable && m_ceilReq < 0);
    m_ceilSpin->setToolTip(m_ceilOk ? tr("Requested ceiling, 48..2047 (default after reset: 1600). Press \"Set ceiling\" to write.") : na);
    m_ceilLbl->setToolTip(m_ceilOk ? tr("Firmware read-back of CAL_motorSpMax: the DShot throttle ceiling applied to every setpoint source. "
                                        "Default 1600 after every reset (flight ceiling, 50 A criterion); raise to 2047 for thrust-rig runs only.")
                                   : na);
    m_ceilBtn->setToolTip(m_ceilOk ? tr("Write CAL_motorSpMax. The tab shows the firmware's read-back, not the requested value.") : na);
}

void MotorView::onSetCeiling()
{
    const int v = m_ceilSpin->value();
    // Write entry point: independent of the widget range.
    if (!m_connected || !m_ceilOk || m_ceilReq >= 0 || !motorCeilingValid(quint32(v)))
        return;
    m_notice.clear();
    m_ceilReq = v;
    m_ceilTimer.start(1500);
    emit logMessage(QStringLiteral("[Motors] set ceiling CAL_motorSpMax = %1 requested").arg(v));
    emit writeCharRequested(QStringLiteral("CAL_motorSpMax"), quint32(v));
    updateCeilingUi();
    updateControls();
}

// Read-back of CAL_motorSpMax. Only the firmware's value is ever shown and
// used for the range, never our own request.
void MotorView::applyCeiling(quint32 value, bool fresh)
{
    if (!m_ceilOk)
        return;
    const bool valid = motorCeilingValid(value);
    const int newCeil = valid ? int(value) : -1;
    if (m_ceilReq >= 0) {
        if (!fresh)
            return;             // a reply to a read issued before our write says nothing about it
        const int req = m_ceilReq;
        m_ceilReq = -1;
        m_ceilTimer.stop();
        if (newCeil == req) {
            m_notice.clear();
            emit logMessage(QStringLiteral("[Motors] ceiling confirmed by read-back: CAL_motorSpMax = %1").arg(req));
        } else {
            m_notice = tr("Ceiling request %1 not in force: the firmware reads back %2.")
                           .arg(req).arg(valid ? QString::number(newCeil) : tr("an invalid value"));
            emit logMessage(QStringLiteral("[Motors] ") + m_notice);
        }
    } else if (newCeil != m_ceiling && m_ceiling >= 0 && valid) {
        emit logMessage(QStringLiteral("[Motors] CAL_motorSpMax read back %1 (was %2) - following the firmware")
                            .arg(newCeil).arg(m_ceiling));
    }
    if (newCeil != m_ceiling && valid)
        m_ceilSpin->setValue(newCeil);   // start the editor at the value in force
    m_ceiling = newCeil;
    applyRange();
}

// ---------------------------------------------------------------------------
// connection / data
// ---------------------------------------------------------------------------

void MotorView::setConnected(bool connected)
{
    m_connected = connected;
    m_haveSample = false;
    m_state = -1;
    m_ceiling = -1;                 // a new connection: the firmware may have reset
    m_ceilReq = -1;
    m_ceilTimer.stop();
    applyRange();
    if (!connected) {
        // Link gone: firmware goes FAILSAFE and zeroes its side; we just reset ours.
        resetManualUi();
        showReadoutsStale();
    } else {
        m_manual = Manual::Unknown;   // until a read-back says otherwise
    }
    updateControls();
    updateBanner();
}

bool MotorView::isStale() const
{
    return !m_connected || !m_haveSample || m_sinceSample.elapsed() > kStaleMs;
}

void MotorView::checkStale()
{
    if (m_connected && m_haveSample && m_sinceSample.elapsed() > kStaleMs) {
        // Grey everything and disable the sliders; state shows STALE and the
        // manual toggle "unknown". Nothing is reset to OFF/STOP -- we do not
        // know that the firmware did. DISARM / STOP ALL stay enabled.
        m_flushTimer.stop();
        for (int &p : m_pending) p = -1;
        m_pendingOp = Pending::None;
        m_pendTimer.stop();
        m_manual = Manual::Unknown;
        showReadoutsStale();
        updateControls();
        updateBanner();
    }
}

void MotorView::feedSample(const XcpClient::Measurements &m)
{
    if (!m_connected)
        return;

    auto dec = [&](int idx, double *out) {
        return idx >= 0 && idx < m_meas.size()
            && A2lModel::decodeFrom(m.blockBase, m.blockRaw, m_meas[idx], out)
            && std::isfinite(*out);
    };

    double sv = 0.0;
    int newState = -1;
    if (m_idxState >= 0) {
        if (!dec(m_idxState, &sv))
            return;   // Esc block not covered yet (first sample); keep waiting
        newState = int(sv);
    }
    const int prevState = m_state;
    m_haveSample = true;
    m_sinceSample.restart();
    m_state = newState;

    // Never fight the firmware: a CONFIRMED exit from ARMED (state read as
    // DISARMED/INIT/FAILSAFE) means it already zeroed manual + setpoints, so
    // mirror that without writing. An unknown state (-1, no EscMotorState in
    // the A2L) is not an exit.
    if (m_state == 0 || m_state == 1 || m_state == 3) {
        resetManualUi();
        m_locked = false;
        m_lockTimer.stop();
    } else if (m_state == 2 && (prevState != 2 || m_manual == Manual::Unknown) && m_charsOk) {
        emit readBackRequested();   // learn CAL_motorManual / Sp now, not in up to 1 s
    }
    if (m_ceilOk && m_ceiling < 0 && m_ceilReq < 0 && m_charsOk)
        emit readBackRequested();   // learn the ceiling at connect, not in up to 1 s

    for (int i = 0; i < kMotors; ++i) {
        const MotorIdx &x = m_idx[i];
        MotorUi &u = m_ui[i];
        double v = 0.0;

        // rpm: bidirectional DShot is the primary source, KISS the fallback.
        bool haveRpm = false;
        if (dec(x.bidirRpm, &v)) {
            u.rpmLbl->setText(QString::number(v, 'f', 0));
            u.rpmCaption->setText(tr("rpm (bidir)"));
            haveRpm = true;
        } else if (dec(x.rpm, &v)) {
            u.rpmLbl->setText(QString::number(v, 'f', 0));
            u.rpmCaption->setText(tr("rpm (KISS fallback)"));
            haveRpm = true;
        }
        if (!haveRpm) {
            u.rpmLbl->setText("--");
            u.rpmCaption->setText(tr("rpm (not in A2L)"));
        }

        // Each source row stands on its own; a missing measurement shows
        // "--" with a tooltip, never a value borrowed from the other source.
        auto show = [&](QLabel *l, int idx, double scale, const QString &suffix, int prec = 0) {
            double d = 0.0;
            if (idx < 0) {
                l->setText("--");
                l->setToolTip(tr("not in this A2L"));
            } else {
                l->setToolTip(QString());
                l->setText(dec(idx, &d) ? QString::number(d * scale, 'f', prec) + suffix : QStringLiteral("--"));
            }
        };
        show(u.erpmLbl,  x.bidirErpm, 1.0,   QString());
        show(u.countsLbl, x.bidirOk,  1.0,   QString());
        show(u.kRpmLbl,  x.rpm,       1.0,   QString());
        show(u.kErpmLbl, x.erpm100,   100.0, QString());
        show(u.tempLbl,  x.temp,      1.0,   QStringLiteral(" °C"));
        {
            double tc = 0, tm = 0;
            if (x.tlmCount < 0 || x.tlmMissed < 0) {
                u.tlmLbl->setText("--");
                u.tlmLbl->setToolTip(tr("not in this A2L"));
            } else {
                u.tlmLbl->setToolTip(QString());
                u.tlmLbl->setText(dec(x.tlmCount, &tc) && dec(x.tlmMissed, &tm)
                                      ? QStringLiteral("%1/%2").arg(tc, 0, 'f', 0).arg(tm, 0, 'f', 0)
                                      : QStringLiteral("--"));
            }
        }

        double cmd = 0.0;
        const bool haveCmd = dec(x.dshot, &cmd);
        u.dshotLbl->setText(haveCmd ? QString::number(cmd, 'f', 0) : QStringLiteral("--"));
        u.bar->setActual(motorBarFraction(cmd), haveCmd);

        if (dec(x.alive, &v)) {
            const bool alive = v != 0.0;
            u.aliveLamp->setPixmap(bulbPixmap(alive ? LampColor::Green : LampColor::Off, alive));
            u.aliveLbl->setText(alive ? tr("alive") : tr("no telemetry"));
        } else {
            u.aliveLamp->setPixmap(bulbPixmap(LampColor::Gray, false));
            u.aliveLbl->setText(tr("--"));
        }

        double vc = 0, ca = 0, mah = 0;
        const bool hV = dec(x.volt, &vc), hC = dec(x.cur, &ca), hM = dec(x.mah, &mah);
        u.rawLbl->setText(QStringLiteral("M%1: %2 cV  %3 cA  %4 mAh")
                              .arg(i + 1)
                              .arg(hV ? QString::number(vc, 'f', 0) : "--",
                                   hC ? QString::number(ca, 'f', 0) : "--",
                                   hM ? QString::number(mah, 'f', 0) : "--"));

        u.rpmLbl->setStyleSheet(QString());
    }

    updateBanner();
    updateControls();
}

void MotorView::showReadoutsStale()
{
    for (int i = 0; i < kMotors; ++i) {
        MotorUi &u = m_ui[i];
        u.rpmLbl->setText("--");
        u.rpmLbl->setStyleSheet(QStringLiteral("QLabel { color: %1; }").arg(kColGrey.name()));
        u.erpmLbl->setText("--");
        u.tempLbl->setText("--");
        u.dshotLbl->setText("--");
        u.countsLbl->setText("--");
        u.kRpmLbl->setText("--");
        u.kErpmLbl->setText("--");
        u.tlmLbl->setText("--");
        u.aliveLbl->setText("--");
        u.aliveLamp->setPixmap(bulbPixmap(LampColor::Gray, false));
        u.bar->setActual(0.0, false);
        u.rawLbl->setText(QStringLiteral("M%1: --").arg(i + 1));
    }
}

// ---------------------------------------------------------------------------
// controls
// ---------------------------------------------------------------------------

bool MotorView::manualWritable() const
{
    return m_connected && m_charsOk && m_readBackProblem.isEmpty() && !isStale() && m_state == 2
        && (!m_ceilOk || m_ceiling >= 0);
}

bool MotorView::slidersEnabled() const
{
    return manualWritable() && m_manual == Manual::On
        && m_pendingOp == Pending::None && !m_locked;
}

QString MotorView::disabledReason() const
{
    if (!m_connected)
        return tr("Not connected.");
    if (!m_charsOk || m_idxState < 0)
        return tr("Not in this A2L (needs EscMotorState and CAL_motorCmd / CAL_motorManual / CAL_motorManualSp_M1..4).");
    if (!m_readBackProblem.isEmpty())
        return tr("Manual throttle disabled: %1.").arg(m_readBackProblem);
    if (isStale())
        return tr("STALE: no ESC sample in the last 1.5 s - manual state unknown, sliders off. DISARM / STOP ALL still work.");
    if (m_state != 2)
        return tr("Arm first (state is %1).").arg(motorStateName(m_state));
    if (m_ceilOk && m_ceiling < 0)
        return tr("Waiting for the CAL_motorSpMax read-back (throttle ceiling unknown).");
    if (m_locked)
        return tr("DISARM sent - waiting for the controller to confirm.");
    if (m_pendingOp == Pending::AwaitZeros)
        return tr("Manual throttle: zeroing the setpoints (waiting for read-back)...");
    if (m_pendingOp == Pending::AwaitOn)
        return tr("Manual throttle: waiting for CAL_motorManual = 1 to be confirmed by read-back...");
    if (m_pendingOp == Pending::AwaitOff)
        return tr("Manual throttle: waiting for CAL_motorManual = 0 to be confirmed by read-back...");
    if (m_manual == Manual::Unknown)
        return tr("Manual throttle state not read back yet.");
    if (m_manual == Manual::Off)
        return tr("Enable manual throttle first.");
    return QString();
}

void MotorView::updateControls()
{
    m_armBtn->setEnabled(m_connected && m_cmdOk && !isStale() && (m_state == 0 || m_state == 3));
    m_disarmBtn->setEnabled(m_connected && m_cmdOk);
    m_stopAllBtn->setEnabled(m_connected && m_charsOk);
    m_manualBtn->setEnabled(manualWritable() && m_pendingOp == Pending::None && !m_locked);
    updateCeilingUi();

    const bool sliders = slidersEnabled();
    m_linkAll->setEnabled(sliders);
    for (int i = 0; i < kMotors; ++i) {
        m_ui[i].slider->setEnabled(sliders);
        m_ui[i].spin->setEnabled(sliders);
        m_ui[i].bar->setMarker(motorBarFraction(motorSliderToDshot(m_ui[i].slider->value(), m_spMax)),
                               sliders);
    }

    const QString why = disabledReason();
    QString text = why.isEmpty()
        ? tr("Manual throttle active: sliders write CAL_motorManualSp_Mn (0 = stop, 48..%1%2).")
              .arg(m_spMax).arg(m_ceilOk ? tr(", firmware ceiling") : QString())
        : why;
    if (!m_notice.isEmpty())
        text = m_notice + QStringLiteral("  ") + text;
    m_hintLbl->setText(text);
    m_hintLbl->setStyleSheet(!m_notice.isEmpty() ? QStringLiteral("QLabel { color: %1; font-weight: bold; }").arg(kColRed.name())
                             : why.isEmpty() ? QString()
                                             : QStringLiteral("QLabel { color: %1; }").arg(kColGrey.name()));
    for (int i = 0; i < kMotors; ++i) {
        m_ui[i].slider->setToolTip(why);
        m_ui[i].spin->setToolTip(why);
    }
    m_linkAll->setToolTip(sliders ? tr("Moving one slider moves all four motors.") : why);
}

void MotorView::updateBanner()
{
    const bool unbound = !m_charsOk || m_idxState < 0;
    m_bindLbl->setVisible(unbound);
    if (unbound)
        m_bindLbl->setText(tr("EscMotorState / CAL_motorCmd / CAL_motorManual / CAL_motorManualSp_M1..4 not (all) in the loaded A2L - "
                              "the Motors tab stays inert until a matching A2L is loaded."));

    QColor c = kColGrey;
    QString text = QStringLiteral("--");
    QString link;
    QColor linkCol = LampColor::Gray;
    if (!m_connected) {
        link = tr("Disconnected");
        linkCol = LampColor::Red;
    } else if (!m_haveSample) {
        link = tr("Connected - waiting for data");
        linkCol = LampColor::Gray;
    } else if (isStale()) {
        link = tr("STALE - no data for >1.5 s");
        linkCol = QColor(224, 160, 0);
        text = QStringLiteral("STALE");
        c = kColAmber;
    } else {
        link = tr("Live");
        linkCol = LampColor::Green;
        text = motorStateName(m_state);
        switch (m_state) {
        case 0: c = kColGrey; break;
        case 1: c = kColAmber; break;
        case 2: c = kColLive; break;
        case 3: c = kColRed; break;
        default: break;
        }
    }
    m_stateLbl->setText(text);
    m_stateLbl->setStyleSheet(QStringLiteral("QLabel { color: %1; }").arg(c.name()));
    m_linkLamp->setPixmap(bulbPixmap(linkCol, true));
    m_linkLbl->setText(link);

    // The toggle shows what the firmware read back -- never our own request.
    QString mtext;
    QString mstyle;
    if (m_pendingOp != Pending::None) {
        mtext = tr("Manual throttle: pending...");
        mstyle = QStringLiteral("QPushButton { background: #f3e2a0; font-style: italic; }");
    } else if (isStale() || m_manual == Manual::Unknown) {
        mtext = tr("Manual throttle: unknown");
        mstyle = QStringLiteral("QPushButton { color: %1; font-style: italic; }").arg(kColGrey.name());
    } else if (m_manual == Manual::On) {
        mtext = tr("Manual throttle: ON");
        mstyle = QStringLiteral("QPushButton { background: #e0a000; font-weight: bold; }");
    } else {
        mtext = tr("Manual throttle: OFF");
    }
    m_manualBtn->setText(mtext);
    m_manualBtn->setStyleSheet(mstyle);
}

void MotorView::setSliderSilently(int motor, int pos)
{
    QSlider *s = m_ui[motor].slider;
    QSpinBox *b = m_ui[motor].spin;
    const QSignalBlocker b1(s), b2(b);
    s->setValue(pos);
    b->setValue(pos);
}

// GUI-only reset: sliders to STOP, manual off, pending writes dropped. Never
// writes. Only for a CONFIRMED exit from ARMED (the firmware has zeroed its
// side) or a dropped link (a write could not happen anyway).
void MotorView::resetManualUi()
{
    m_flushTimer.stop();
    for (int &p : m_pending) p = -1;
    m_manual = Manual::Off;
    m_pendingOp = Pending::None;
    m_pendTimer.stop();
    m_notice.clear();
    for (int i = 0; i < kMotors; ++i)
        setSliderSilently(i, kMotorSliderStop);
    if (m_linkAll)
        m_linkAll->setChecked(false);
    for (int i = 0; i < kMotors; ++i)
        m_ui[i].bar->setMarker(0.0, false);
}

void MotorView::onArmClicked()
{
    const auto answer = QMessageBox::question(
        this, tr("Arm motors?"),
        tr("This arms the motors (CAL_motorCmd = 1). They will spin at any "
           "nonzero throttle command from this point on.\n\nArm?"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer == QMessageBox::Yes && m_connected)
        emit armRequested();
}

void MotorView::writeAllStop()
{
    for (int i = 0; i < kMotors; ++i)
        emit writeCharRequested(spName(i), 0);
}

// Operator asked for manual ON/OFF. Nothing is believed until the firmware
// reads it back:
//   ON : Sp=0 x4, read back, only if all four read 0 -> CAL_motorManual=1,
//        read back, only if it reads 1 -> sliders live.
//   OFF: Sp=0 x4, CAL_motorManual=0, read back must show 0.
void MotorView::onManualClicked(bool on)
{
    if (!manualWritable() || m_pendingOp != Pending::None || m_locked) {
        updateBanner();   // raced with a state change: write nothing
        return;
    }
    m_flushTimer.stop();
    for (int &p : m_pending) p = -1;
    for (int i = 0; i < kMotors; ++i)
        setSliderSilently(i, kMotorSliderStop);
    m_notice.clear();
    writeAllStop();
    if (on) {
        m_pendingOp = Pending::AwaitZeros;
        m_pendTimer.start(1000);
    } else {
        emit writeCharRequested(QStringLiteral("CAL_motorManual"), 0u);
        m_pendingOp = Pending::AwaitOff;
        m_pendTimer.start(1500);
    }
    emit readBackRequested();
    updateBanner();
    updateControls();
}

void MotorView::abortPending(const QString &why)
{
    const Pending was = m_pendingOp;
    m_pendingOp = Pending::None;
    m_pendTimer.stop();
    m_notice = why;
    emit logMessage(QStringLiteral("[Motors] ") + why);
    // After a failed ON nothing was written for manual (still Off as before);
    // after a failed ON/OFF confirmation the firmware state is unknown until
    // the next read-back.
    if (was != Pending::AwaitZeros)
        m_manual = Manual::Unknown;
    emit readBackRequested();
    updateBanner();
    updateControls();
}

void MotorView::pendingTimeout()
{
    switch (m_pendingOp) {
    case Pending::AwaitZeros:
        abortPending(tr("Manual ON aborted: the setpoint read-back did not show four zeros within 1 s; CAL_motorManual=1 was NOT written."));
        break;
    case Pending::AwaitOn:
        abortPending(tr("CAL_motorManual=1 was not confirmed by read-back within 1.5 s (write failed?) - manual throttle not enabled."));
        break;
    case Pending::AwaitOff:
        abortPending(tr("CAL_motorManual=0 was not confirmed by read-back within 1.5 s - manual state unknown."));
        break;
    case Pending::None:
        break;
    }
}

// Drop pending slider writes and lock the sliders; the reset to STOP/OFF
// happens on the CONFIRMED exit from ARMED (EscMotorState), not on the click.
void MotorView::onDisarmIssued()
{
    m_flushTimer.stop();
    for (int &p : m_pending) p = -1;
    m_pendingOp = Pending::None;
    m_pendTimer.stop();
    m_locked = true;
    m_lockTimer.start();
    updateControls();
    updateBanner();
}

void MotorView::setReadBackProblem(const QString &reason)
{
    m_readBackProblem = reason;
    updateControls();
}

void MotorView::onCharsRead(const QMap<QString, quint32> &v, bool fresh)
{
    if (m_connected && v.contains(QStringLiteral("CAL_motorSpMax"))) {
        applyCeiling(v.value(QStringLiteral("CAL_motorSpMax")), fresh);
        updateControls();
    }
    if (!m_connected || m_state != 2 || !v.contains(QStringLiteral("CAL_motorManual")))
        return;
    const bool man = v.value(QStringLiteral("CAL_motorManual")) == 1u;
    bool spZero = true;
    for (int i = 0; i < kMotors; ++i)
        spZero = spZero && v.contains(spName(i)) && v.value(spName(i)) == 0u;

    if (!fresh && m_pendingOp != Pending::None) {
        // A reply to a read issued before our last write says nothing about
        // that write -- ask again instead of accepting it.
        QTimer::singleShot(150, this, [this]() {
            if (m_pendingOp != Pending::None)
                emit readBackRequested();
        });
        return;
    }
    switch (m_pendingOp) {
    case Pending::AwaitZeros:
        if (spZero) {
            m_pendingOp = Pending::AwaitOn;
            m_pendTimer.start(1500);
            emit writeCharRequested(QStringLiteral("CAL_motorManual"), 1u);
            emit readBackRequested();
        } else {
            QTimer::singleShot(150, this, [this]() {
                if (m_pendingOp == Pending::AwaitZeros)
                    emit readBackRequested();
            });
        }
        break;
    case Pending::AwaitOn:
        // A read issued before our write landed shows 0 -- only the timeout aborts.
        if (man) {
            m_pendingOp = Pending::None;
            m_pendTimer.stop();
            m_manual = Manual::On;
        } else {
            QTimer::singleShot(150, this, [this]() {
                if (m_pendingOp == Pending::AwaitOn)
                    emit readBackRequested();
            });
        }
        break;
    case Pending::AwaitOff:
        if (!man) {
            m_pendingOp = Pending::None;
            m_pendTimer.stop();
            m_manual = Manual::Off;
        } else {
            QTimer::singleShot(150, this, [this]() {
                if (m_pendingOp == Pending::AwaitOff)
                    emit readBackRequested();
            });
        }
        break;
    case Pending::None:
        if (man && m_manual != Manual::On) {
            // Enabled (or still enabled) in the firmware: adopt its setpoints.
            m_manual = Manual::On;
            m_notice.clear();
            for (int i = 0; i < kMotors; ++i)
                if (v.contains(spName(i)))
                    setSliderSilently(i, motorDshotToSlider(int(v.value(spName(i))), m_spMax));
        } else if (!man && m_manual != Manual::Off) {
            if (m_manual == Manual::On) {
                m_notice = tr("Manual throttle was switched off outside this tab (CAL_motorManual reads 0) - following the firmware.");
                emit logMessage(QStringLiteral("[Motors] CAL_motorManual read back 0 while the GUI showed ON - following the firmware"));
            }
            m_manual = Manual::Off;
            m_flushTimer.stop();
            for (int &p : m_pending) p = -1;
            for (int i = 0; i < kMotors; ++i)
                setSliderSilently(i, kMotorSliderStop);
        }
        break;
    }
    updateBanner();
    updateControls();
}

void MotorView::onStopAll()
{
    m_flushTimer.stop();
    for (int &p : m_pending) p = -1;
    for (int i = 0; i < kMotors; ++i)
        setSliderSilently(i, kMotorSliderStop);
    if (m_connected && m_charsOk)
        emit stopAllRequested();   // XcpPanel: purge queued setpoints, priority Sp=0 x4
    updateControls();
}

void MotorView::onSliderChanged(int motor, int pos)
{
    {
        const QSignalBlocker b(m_ui[motor].spin);
        m_ui[motor].spin->setValue(pos);
    }
    if (m_linkAll->isChecked() && !m_inLink) {
        m_inLink = true;
        for (int j = 0; j < kMotors; ++j) {
            if (j == motor) continue;
            setSliderSilently(j, pos);
            m_pending[j] = pos;
        }
        m_inLink = false;
    }
    m_pending[motor] = pos;
    if (!m_flushTimer.isActive())
        m_flushTimer.start();   // rate-limit to ~20 Hz; the latest value always goes out
    updateControls();
}

void MotorView::onSpinChanged(int motor, int pos)
{
    // Commit from the spinbox (Enter / arrows): route through the slider so
    // link-all, pending coalescing and the marker all follow one path.
    m_ui[motor].slider->setValue(pos);
}

void MotorView::flushPending()
{
    // The writes only happen while everything is in order; otherwise drop.
    const bool ok = slidersEnabled();
    for (int i = 0; i < kMotors; ++i) {
        if (m_pending[i] < 0)
            continue;
        const int dshot = motorSliderToDshot(m_pending[i], m_spMax);
        m_pending[i] = -1;
        if (!ok || !motorDshotWritable(quint32(dshot), m_spMax))
            continue;
        emit writeCharRequested(spName(i), quint32(dshot));
    }
}
