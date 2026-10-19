// Headless widget-level self-check of the Motors tab's SAFETY logic.
// Instantiates MotorView on the offscreen platform with a synthetic A2L
// (EscMotorState + CAL_motor* characteristics) and synthetic samples /
// read-backs, and asserts what it may and may not write. No XCP, no board.
// Built and run by build.bat and CI; any [FAIL] line = non-zero exit.
#include "motorview.h"
#include "motormath.h"

#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QCheckBox>
#include <QTimer>
#include <cstdio>

namespace {

int g_fail = 0;
void check(const char *name, bool ok)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

void wait(int ms)
{
    QEventLoop l;
    QTimer::singleShot(ms, &l, &QEventLoop::quit);
    l.exec();
}

struct Rig {
    MotorView view;
    QVector<QPair<QString, quint32>> writes;
    int disarms = 0, stopAlls = 0, readBacks = 0, arms = 0;
    QString lastLog;

    Rig()
    {
        QVector<A2lMeas> meas;
        A2lMeas st; st.name = "EscMotorState"; st.addr = 0x1000; st.type = A2lType::Uint8; st.hi = 3;
        meas.append(st);
        QVector<A2lChar> chars;
        auto add = [&](const char *n, quint32 a, A2lType t, double hi) {
            A2lChar c; c.name = n; c.addr = a; c.type = t; c.hi = hi; chars.append(c);
        };
        add("CAL_motorCmd", 0x2140, A2lType::Uint32, 1);
        add("CAL_motorManual", 0x2144, A2lType::Uint32, 1);
        add("CAL_motorManualSp_M1", 0x2148, A2lType::Uint16, 2047);
        add("CAL_motorManualSp_M2", 0x214A, A2lType::Uint16, 2047);
        add("CAL_motorManualSp_M3", 0x214C, A2lType::Uint16, 2047);
        add("CAL_motorManualSp_M4", 0x214E, A2lType::Uint16, 2047);
        view.setAvailable(meas);
        view.setCharsAvailable(chars);
        QObject::connect(&view, &MotorView::writeCharRequested, &view,
                         [this](const QString &n, quint32 v) { writes.append({n, v}); });
        QObject::connect(&view, &MotorView::disarmRequested, &view, [this]() { ++disarms; });
        QObject::connect(&view, &MotorView::stopAllRequested, &view, [this]() { ++stopAlls; });
        QObject::connect(&view, &MotorView::armRequested, &view, [this]() { ++arms; });
        QObject::connect(&view, &MotorView::readBackRequested, &view, [this]() { ++readBacks; });
        QObject::connect(&view, &MotorView::logMessage, &view, [this](const QString &t) { lastLog = t; });
    }

    // Variant with CAL_motorSpMax in the A2L (call before connecting)
    void addCeilingChar()
    {
        QVector<A2lMeas> meas;
        A2lMeas st; st.name = "EscMotorState"; st.addr = 0x1000; st.type = A2lType::Uint8; st.hi = 3;
        meas.append(st);
        QVector<A2lChar> chars;
        auto add = [&](const char *n, quint32 a, A2lType t, double hi) {
            A2lChar c; c.name = n; c.addr = a; c.type = t; c.hi = hi; chars.append(c);
        };
        add("CAL_motorCmd", 0x2140, A2lType::Uint32, 1);
        add("CAL_motorManual", 0x2144, A2lType::Uint32, 1);
        for (int i = 0; i < 4; ++i)
            add(qPrintable(QString("CAL_motorManualSp_M%1").arg(i + 1)), 0x2148 + 2 * i, A2lType::Uint16, 2047);
        add("CAL_motorSpMax", 0x2150, A2lType::Uint16, 2047);
        view.setAvailable(meas);
        view.setCharsAvailable(chars);
    }
    void readBackCeil(quint32 manual, quint32 ceil, bool fresh = true)
    {
        QMap<QString, quint32> v;
        v["CAL_motorCmd"] = 1;
        v["CAL_motorManual"] = manual;
        for (int i = 1; i <= 4; ++i)
            v[QString("CAL_motorManualSp_M%1").arg(i)] = 0;
        v["CAL_motorSpMax"] = ceil;
        view.onCharsRead(v, fresh);
    }
    QPushButton *ceilBtn() { return button("Set ceiling", true); }
    QSpinBox *ceilSpin()
    {
        for (QSpinBox *b : view.findChildren<QSpinBox *>())
            if (b->minimum() == kMotorDshotMin) return b;
        return nullptr;
    }
    void sample(int state)
    {
        XcpClient::Measurements m;
        m.blockBase = { 0x1000 };
        m.blockRaw = { QByteArray(1, char(state)) };
        view.feedSample(m);
    }
    void readBack(quint32 manual, quint32 sp = 0)
    {
        QMap<QString, quint32> v;
        v["CAL_motorCmd"] = 1;
        v["CAL_motorManual"] = manual;
        for (int i = 1; i <= 4; ++i)
            v[QString("CAL_motorManualSp_M%1").arg(i)] = sp;
        view.onCharsRead(v);
    }
    QPushButton *button(const QString &prefix, bool exact = false)
    {
        for (QPushButton *b : view.findChildren<QPushButton *>())
            if (exact ? b->text() == prefix : b->text().startsWith(prefix))
                return b;
        return nullptr;
    }
    QVector<QSlider *> sliders() { return view.findChildren<QSlider *>().toVector(); }
    bool slidersEnabled()
    {
        for (QSlider *s : sliders())
            if (s->isEnabled()) return true;
        return false;
    }
    bool hasText(const QString &needle)
    {
        for (QLabel *l : view.findChildren<QLabel *>())
            if (l->text().contains(needle)) return true;
        return false;
    }
    int countWrites(const QString &name, int from = 0) const
    {
        int n = 0;
        for (int i = from; i < writes.size(); ++i)
            if (writes[i].first == name) ++n;
        return n;
    }
    // connect, ARMED, manual enabled through the full handshake
    void goManualOn()
    {
        view.setConnected(true);
        sample(2);
        readBack(0);
        button("Manual throttle")->click();
        readBack(0, 0);          // four zeros read back -> manual=1 is written
        readBack(1, 0);          // manual=1 confirmed
    }
};

} // namespace

int main(int argc, char **argv)
{
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);

    // 1. nothing is written while disconnected; ARM off while state unknown
    {
        Rig r;
        r.sample(2);   // ignored: not connected
        check("disconnected: ARM disabled", !r.button("ARM", true)->isEnabled());
        check("disconnected: sliders disabled", !r.slidersEnabled());
        r.sliders()[0]->setValue(500);
        wait(120);
        check("disconnected: no write", r.writes.isEmpty());
        r.view.setConnected(true);
        check("connected, no sample yet: ARM disabled (state unknown)", !r.button("ARM", true)->isEnabled());
        r.sample(0);
        check("DISARMED fresh: ARM enabled", r.button("ARM", true)->isEnabled());
        check("DISARMED: sliders and manual disabled", !r.slidersEnabled() && !r.button("Manual throttle")->isEnabled());
    }

    // 2. sliders only after read-back manual=1 AND ARMED; handshake order
    {
        Rig r;
        r.view.setConnected(true);
        r.sample(2);
        check("ARMED, manual unknown: read-back requested", r.readBacks > 0);
        check("ARMED, manual unknown: sliders disabled", !r.slidersEnabled());
        r.readBack(0);
        check("read-back manual=0: sliders disabled", !r.slidersEnabled());
        r.button("Manual throttle")->click();
        check("Manual click: four Sp=0 written", r.writes.size() == 4 && r.countWrites("CAL_motorManual") == 0);
        bool allZero = true;
        for (auto &w : r.writes) allZero = allZero && w.second == 0;
        check("Manual click: those writes are zeros, manual NOT yet written", allZero);
        r.readBack(0, 100);   // Sp not zero
        check("Sp read-back non-zero: manual=1 still not written", r.countWrites("CAL_motorManual") == 0);
        {
            QMap<QString, quint32> z;
            z["CAL_motorManual"] = 0;
            for (int i = 1; i <= 4; ++i) z[QString("CAL_motorManualSp_M%1").arg(i)] = 0;
            r.view.onCharsRead(z, /*fresh=*/false);   // reply to a read issued BEFORE the writes
            check("stale (pre-write) zeros reply does not confirm: manual=1 still not written",
                  r.countWrites("CAL_motorManual") == 0);
        }
        r.readBack(0, 0);
        check("Sp read-back four zeros: manual=1 written after the Sp zeros",
              r.writes.size() == 5 && r.writes.last().first == "CAL_motorManual" && r.writes.last().second == 1);
        check("manual written but unconfirmed: sliders still disabled", !r.slidersEnabled());
        r.readBack(1, 0);
        check("read-back manual=1 + ARMED: sliders enabled", r.slidersEnabled());
    }

    // 3. zeros never confirmed -> abort after 1 s, manual=1 never written
    {
        Rig r;
        r.view.setConnected(true);
        r.sample(2);
        r.readBack(0);
        r.button("Manual throttle")->click();
        wait(1250);
        check("no zero read-back in 1 s: manual=1 NOT written", r.countWrites("CAL_motorManual") == 0);
        check("... aborted with a log line and a visible notice", !r.lastLog.isEmpty() && r.hasText("NOT written"));
        check("... sliders still disabled", !r.slidersEnabled());
    }

    // 4. exit from ARMED resets the GUI without any write
    {
        Rig r;
        r.goManualOn();
        r.sliders()[0]->setValue(500);
        wait(120);
        const int before = r.writes.size();
        r.sample(0);
        check("ARMED -> DISARMED: no write", r.writes.size() == before);
        check("ARMED -> DISARMED: sliders reset to STOP and disabled",
              r.sliders()[0]->value() == kMotorSliderStop && !r.slidersEnabled());
        check("ARMED -> DISARMED: toggle shows OFF", r.button("Manual throttle")->text().contains("OFF"));
    }

    // 5. coalescing: only the latest value goes out
    {
        Rig r;
        r.goManualOn();
        r.writes.clear();
        QSlider *s = r.sliders()[0];
        s->setValue(100); s->setValue(200); s->setValue(300);
        wait(150);
        check("coalescing: exactly one M1 write", r.countWrites("CAL_motorManualSp_M1") == 1);
        check("coalescing: it is the latest value (300)", r.writes.size() == 1 && r.writes[0].second == 300);
    }

    // 6. keys: End/Home/PageUp/PageDown swallowed, Up = single step
    {
        Rig r;
        r.goManualOn();
        r.writes.clear();
        QSlider *s = r.sliders()[0];
        for (int key : { Qt::Key_End, Qt::Key_Home, Qt::Key_PageUp, Qt::Key_PageDown }) {
            QKeyEvent e(QEvent::KeyPress, key, Qt::NoModifier);
            QCoreApplication::sendEvent(s, &e);
        }
        wait(120);
        check("End/Home/PageUp/PageDown change nothing", s->value() == kMotorSliderStop && r.writes.isEmpty());
        QKeyEvent up(QEvent::KeyPress, Qt::Key_Up, Qt::NoModifier);
        QCoreApplication::sendEvent(s, &up);
        check("Up arrow steps by exactly 1", s->value() == kMotorSliderStop + 1);
    }

    // 7. DISARM / STOP ALL drop pending slider writes
    {
        Rig r;
        r.goManualOn();
        r.writes.clear();
        r.sliders()[1]->setValue(900);
        r.button("DISARM", true)->click();
        wait(150);
        check("DISARM: pending setpoint dropped, nothing queued by the view", r.writes.isEmpty() && r.disarms == 1);
        check("DISARM: sliders locked until the firmware confirms", !r.slidersEnabled());
    }
    {
        Rig r;
        r.goManualOn();
        r.writes.clear();
        r.sliders()[1]->setValue(900);
        r.button("STOP ALL", true)->click();
        wait(150);
        check("STOP ALL: pending setpoint dropped, one stopAll request", r.writes.isEmpty() && r.stopAlls == 1);
        check("STOP ALL: sliders at STOP", r.sliders()[1]->value() == kMotorSliderStop);
    }

    // 7b. a footer DISARM (XcpPanel -> onDisarmIssued) locks like the tab's own button
    {
        Rig r;
        r.goManualOn();
        r.writes.clear();
        r.sliders()[0]->setValue(900);
        r.view.onDisarmIssued();
        wait(150);
        check("external DISARM: pending setpoint dropped, sliders locked", r.writes.isEmpty() && !r.slidersEnabled());
    }

    // 8. A2L reload keeps state and values
    {
        Rig r;
        r.goManualOn();
        r.sliders()[2]->setValue(777);
        wait(120);
        QVector<A2lMeas> meas;
        A2lMeas st; st.name = "EscMotorState"; st.addr = 0x1000; st.type = A2lType::Uint8; st.hi = 3;
        meas.append(st);
        r.view.setAvailable(meas);
        check("A2L reload: slider value kept", r.sliders()[2]->value() == 777);
        check("A2L reload: still enabled", r.slidersEnabled());
    }

    // 9. firmware says manual=0 while the GUI showed ON -> follow it
    {
        Rig r;
        r.goManualOn();
        r.readBack(0);
        check("read-back 0 while ON: sliders disabled, toggle OFF, reason shown",
              !r.slidersEnabled() && r.button("Manual throttle")->text().contains("OFF")
                  && r.hasText("switched off outside"));
    }

    // 10. stale: unknown, never OFF; DISARM / STOP ALL stay available
    {
        Rig r;
        r.goManualOn();
        wait(2200);
        check("stale: sliders disabled", !r.slidersEnabled());
        check("stale: toggle 'unknown', not OFF", r.button("Manual throttle")->text().contains("unknown"));
        check("stale: DISARM and STOP ALL enabled",
              r.button("DISARM", true)->isEnabled() && r.button("STOP ALL", true)->isEnabled());
        check("stale: state banner says STALE", r.hasText("STALE"));
    }

    // 11. DShot ceiling (CAL_motorSpMax): range follows the read-back
    {
        Rig r;
        r.addCeilingChar();
        r.view.setConnected(true);
        r.sample(2);
        check("ceiling: read-back requested at connect", r.readBacks > 0);
        r.readBackCeil(0, 1600);
        check("ceiling 1600: label shows the firmware value", r.hasText("Ceiling (CAL_motorSpMax):") && r.hasText("1600"));
        check("ceiling 1600: every slider maximum 1600, minimum still 47",
              r.sliders().size() == 4 && r.sliders()[0]->maximum() == 1600 && r.sliders()[3]->maximum() == 1600
                  && r.sliders()[0]->minimum() == kMotorSliderStop);
        check("ceiling 1600: spinboxes follow", [&]() {
            for (QSpinBox *b : r.view.findChildren<QSpinBox *>())
                if (b->minimum() == kMotorSliderStop && b->maximum() != 1600) return false;
            return true; }());
        check("ceiling: editor starts at the value in force", r.ceilSpin() && r.ceilSpin()->value() == 1600);
        // manual ON, link-all drag beyond the ceiling is impossible
        r.button("Manual throttle")->click();
        r.readBackCeil(0, 1600);
        r.readBackCeil(1, 1600);
        check("ceiling 1600: sliders enabled after manual handshake", r.slidersEnabled());
        r.writes.clear();
        r.view.findChild<QCheckBox *>()->setChecked(true);
        r.sliders()[0]->setValue(2047);   // clamps to the maximum
        wait(120);
        check("ceiling 1600: link-all at the top writes 1600 on all four, never more",
              r.writes.size() == 4 && [&]() { for (auto &w : r.writes) if (w.second != 1600) return false; return true; }());
        // Set ceiling 2047: not believed until read back
        r.writes.clear();
        r.ceilSpin()->setValue(2047);
        r.ceilBtn()->click();
        check("ceiling: Set ceiling writes CAL_motorSpMax = 2047",
              r.writes.size() == 1 && r.writes[0].first == "CAL_motorSpMax" && r.writes[0].second == 2047);
        check("ceiling: range unchanged until the read-back", r.sliders()[0]->maximum() == 1600);
        r.readBackCeil(1, 1600, false);   // stale reply: ignored
        check("ceiling: stale reply does not confirm", r.sliders()[0]->maximum() == 1600 && r.hasText("pending"));
        r.readBackCeil(1, 2047);
        check("ceiling 2047: slider maximum 2047 after read-back", r.sliders()[0]->maximum() == 2047);
        // request 1800, firmware keeps 2047: the read-back wins and the tab says so
        r.ceilSpin()->setValue(1800);
        r.ceilBtn()->click();
        r.readBackCeil(1, 2047);
        check("ceiling: firmware kept another value -> shown with a notice, range = firmware value",
              r.sliders()[0]->maximum() == 2047 && r.hasText("not in force"));
    }

    // 11b. lowering the ceiling under a live slider rewrites the setpoint
    {
        Rig r;
        r.addCeilingChar();
        r.view.setConnected(true);
        r.sample(2);
        r.readBackCeil(0, 2047);
        r.button("Manual throttle")->click();
        r.readBackCeil(0, 2047);
        r.readBackCeil(1, 2047);
        r.sliders()[1]->setValue(1800);
        wait(120);
        r.writes.clear();
        r.readBackCeil(1, 1600);
        wait(120);
        check("ceiling lowered under slider 1800: slider shows 1600", r.sliders()[1]->value() == 1600);
        check("ceiling lowered under slider 1800: write of 1600 to M2",
              r.countWrites("CAL_motorManualSp_M2") == 1 && r.writes.size() == 1 && r.writes[0].second == 1600);
    }

    // 11c. ceiling unknown / disconnect / unconfirmed
    {
        Rig r;
        r.addCeilingChar();
        r.view.setConnected(true);
        r.sample(2);
        QMap<QString, quint32> v;   // read-back without the ceiling
        v["CAL_motorCmd"] = 1; v["CAL_motorManual"] = 0;
        for (int i = 1; i <= 4; ++i) v[QString("CAL_motorManualSp_M%1").arg(i)] = 0;
        r.view.onCharsRead(v);
        check("ARMED, ceiling unknown: Manual and sliders not writable",
              !r.button("Manual throttle")->isEnabled() && !r.slidersEnabled());
        r.readBackCeil(0, 1600);
        check("ceiling known: Manual writable", r.button("Manual throttle")->isEnabled());
        r.view.setConnected(false);
        r.view.setConnected(true);
        r.sample(2);
        check("reconnect: ceiling unknown again (label --, Manual not writable)",
              r.hasText("Ceiling (CAL_motorSpMax): --") && !r.button("Manual throttle")->isEnabled());
        r.readBackCeil(0, 1600);
        r.ceilBtn()->click();
        wait(1700);
        r.sample(2);
        check("no confirmation in 1.5 s: announced as not confirmed", r.hasText("not confirmed"));
    }

    // 12. ceiling write while DISARMED, and fallback without CAL_motorSpMax
    {
        Rig r;
        r.addCeilingChar();
        r.view.setConnected(true);
        r.sample(0);
        r.readBackCeil(0, 1600);
        r.ceilSpin()->setValue(r.ceilSpin()->minimum());
        r.ceilBtn()->click();
        check("ceiling: 48 is writable (DISARMED is fine)", r.writes.size() == 1 && r.writes[0].second == 48);
    }
    {
        Rig r;   // default Rig: A2L without CAL_motorSpMax
        r.view.setConnected(true);
        r.sample(2);
        check("fallback: slider range 47..2047", r.sliders()[0]->maximum() == 2047);
        check("fallback: ceiling button and spinbox disabled", r.ceilBtn() && !r.ceilBtn()->isEnabled()
                                                               && r.ceilSpin() && !r.ceilSpin()->isEnabled());
        check("fallback: tooltip names the cause", r.ceilBtn()->toolTip() == "firmware without CAL_motorSpMax");
        r.writes.clear();
        r.ceilBtn()->click();
        check("fallback: clicking writes nothing", r.writes.isEmpty());
        r.readBack(0);
        r.button("Manual throttle")->click();
        r.readBack(0, 0);
        r.readBack(1, 0);
        check("fallback: manual handshake works as before", r.slidersEnabled());
    }

    std::printf("%s\n", g_fail == 0 ? "[motor_view_selfcheck] PASS" : "[motor_view_selfcheck] FAIL");
    return g_fail == 0 ? 0 : 1;
}
