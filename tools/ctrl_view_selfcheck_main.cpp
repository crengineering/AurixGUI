// Headless widget-level self-check of the flight-controller panel (Attitude
// tab): CtrlView on the offscreen platform with a synthetic A2L and synthetic
// samples / read-backs. Asserts what it may and may not write, that the values
// shown are the read-back, and that trips / drops are reported. No XCP, no board.
// Built and run by build.bat and CI; any [FAIL] line = non-zero exit.
#include "ctrlview.h"

#include <QApplication>
#include <QEventLoop>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QtEndian>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cmath>

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

// Synthetic measurement block at 0x1000.
constexpr quint32 kBase = 0x1000;
constexpr int kOffState = 0x00, kOffStatus = 0x04, kOffTrips = 0x08, kOffRoll = 0x0C, kOffSp0 = 0x10;

struct Rig {
    CtrlView view;
    QVector<QPair<QString, double>> writes;
    int readBacks = 0;
    QString lastLog;

    Rig()
    {
        QVector<A2lMeas> meas;
        auto m = [&](const char *n, int off, A2lType t) {
            A2lMeas x; x.name = n; x.addr = kBase + quint32(off); x.type = t; meas.append(x);
        };
        m("EscMotorState", kOffState, A2lType::Uint8);
        m("CtrlStatus", kOffStatus, A2lType::Uint32);
        m("CtrlTrips", kOffTrips, A2lType::Uint32);
        m("AttRoll", kOffRoll, A2lType::Float32);
        m("CtrlAttSp_0", kOffSp0, A2lType::Float32);
        view.setAvailable(meas);

        QVector<A2lChar> chars;
        auto c = [&](const char *n, quint32 a, A2lType t, double lo, double hi) {
            A2lChar x; x.name = n; x.addr = a; x.type = t; x.lo = lo; x.hi = hi; chars.append(x);
        };
        c("CAL_ctrlMode", 0x2154, A2lType::Uint32, 0, 1);
        c("CAL_ctrlThrustN", 0x2158, A2lType::Float32, 0, 40);
        c("CAL_ctrlTiltMaxDeg", 0x215C, A2lType::Float32, 0, 90);
        c("CAL_ctrlRpmMin", 0x2160, A2lType::Float32, 0, 3000);
        c("CAL_ctrlAttSp_0", 0x2188, A2lType::Float32, -90, 360);
        c("CAL_ctrlAttSp_1", 0x218C, A2lType::Float32, -90, 360);
        c("CAL_ctrlAttSp_2", 0x2190, A2lType::Float32, -90, 360);
        view.setCharsAvailable(chars);

        QObject::connect(&view, &CtrlView::writeCharRequested, &view,
                         [this](const QString &n, double v) { writes.append({n, v}); });
        QObject::connect(&view, &CtrlView::readBackRequested, &view, [this]() { ++readBacks; });
        QObject::connect(&view, &CtrlView::logMessage, &view, [this](const QString &t) { lastLog = t; });
    }

    void sample(int state, quint32 status = 0, quint32 trips = 0, float roll = 0.0f, float sp0 = 0.0f)
    {
        QByteArray b(0x20, '\0');
        b[kOffState] = char(state);
        qToLittleEndian<quint32>(status, b.data() + kOffStatus);
        qToLittleEndian<quint32>(trips, b.data() + kOffTrips);
        std::memcpy(b.data() + kOffRoll, &roll, 4);
        std::memcpy(b.data() + kOffSp0, &sp0, 4);
        XcpClient::Measurements mm;
        mm.blockBase = { kBase };
        mm.blockRaw = { b };
        view.feedSample(mm);
    }
    void readBack(double mode, double r = 0, double p = 0, double y = -1, double thrust = 0,
                  double tilt = 30, bool fresh = true)
    {
        QMap<QString, double> v;
        v["CAL_ctrlMode"] = mode;
        v["CAL_ctrlThrustN"] = thrust;
        v["CAL_ctrlTiltMaxDeg"] = tilt;
        v["CAL_ctrlRpmMin"] = 1300;
        view.onCharsRead(v, fresh);
        QMap<QString, double> s;
        s["CAL_ctrlAttSp_0"] = r;
        s["CAL_ctrlAttSp_1"] = p;
        s["CAL_ctrlAttSp_2"] = y;
        view.onCharsRead(s, fresh);
    }
    QPushButton *button(const QString &prefix)
    {
        for (QPushButton *b : view.findChildren<QPushButton *>())
            if (b->text().startsWith(prefix))
                return b;
        return nullptr;
    }
    bool hasText(const QString &needle)
    {
        for (QLabel *l : view.findChildren<QLabel *>())
            if (l->text().contains(needle))
                return true;
        return false;
    }
    bool bannerVisibleWith(const QString &needle)
    {
        for (QLabel *l : view.findChildren<QLabel *>())
            if (!l->isHidden() && l->text().contains(needle) && l->styleSheet().contains("c0392b"))
                return true;
        return false;
    }
    QDoubleSpinBox *spin(int i) { return view.findChildren<QDoubleSpinBox *>().value(i); }
    int nWrites(const char *name)
    {
        int n = 0;
        for (const auto &w : writes) if (w.first == name) ++n;
        return n;
    }
    // arm the rig: connected, ARMED, firmware read-back shadow / 0 0 -1
    void armed()
    {
        view.setConnected(true);
        sample(2);
        readBack(0);
    }
};

// Answer the next modal QMessageBox with Yes or No.
void answerDialog(bool yes)
{
    QTimer::singleShot(150, [yes]() {
        if (auto *box = qobject_cast<QMessageBox *>(QApplication::activeModalWidget()))
            box->button(yes ? QMessageBox::Yes : QMessageBox::No)->click();
    });
}

} // namespace

int main(int argc, char **argv)
{
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);

    // 1. disconnected / disarmed: nothing can be written, reason visible
    {
        Rig r;
        r.sample(2);   // ignored: not connected
        check("disconnected: Apply, Level and Drive disabled",
              !r.button("Apply")->isEnabled() && !r.button("Level")->isEnabled() && !r.button("Drive")->isEnabled());
        check("disconnected: Shadow disabled (no link)", !r.button("Shadow")->isEnabled());
        r.view.setConnected(true);
        r.sample(0);
        r.readBack(0);
        check("DISARMED: entry disabled with the state named", !r.button("Apply")->isEnabled() && r.hasText("DISARMED"));
        r.button("Apply")->click();
        r.button("Level")->click();
        check("DISARMED: clicks write nothing", r.writes.isEmpty());
        check("DISARMED: Drive disabled", !r.button("Drive")->isEnabled());
    }

    // 2. ARMED: Apply writes three words, confirmed only by a FRESH read-back
    {
        Rig r;
        r.armed();
        check("ARMED + read-back: entry and Drive enabled", r.button("Apply")->isEnabled() && r.button("Drive")->isEnabled());
        r.spin(0)->setValue(10.0);
        r.spin(1)->setValue(-5.0);
        r.button("Apply")->click();
        check("Apply: roll, pitch, yaw(-1 = hold) written in that order",
              r.writes.size() == 3 && r.writes[0] == qMakePair(QString("CAL_ctrlAttSp_0"), 10.0)
              && r.writes[1] == qMakePair(QString("CAL_ctrlAttSp_1"), -5.0)
              && r.writes[2] == qMakePair(QString("CAL_ctrlAttSp_2"), -1.0));
        check("Apply: shows pending, not the typed value, and locks the entry",
              r.hasText("pending...") && !r.button("Apply")->isEnabled());
        r.readBack(0, 0, 0, -1, 0, 30, /*fresh=*/false);
        check("stale read-back does not confirm", r.hasText("pending..."));
        r.readBack(0, 10, -5, -1);
        check("fresh read-back confirms: 'in force' shows the read-back",
              !r.hasText("pending...") && r.hasText("in force: 10.0 deg") && r.hasText("in force: -5.0 deg")
              && r.hasText("in force: hold") && r.button("Apply")->isEnabled());
    }

    // 3. gates and clamps are explained, never silent
    {
        Rig r;
        r.armed();
        r.spin(0)->setValue(45.0);   // firmware limit 30
        r.button("Apply")->click();
        check("roll above the tilt limit: nothing sent, message names the limit",
              r.writes.isEmpty() && r.bannerVisibleWith("tilt limit of +-30.0"));
        r.spin(0)->setValue(20.0);
        r.button("Apply")->click();
        check("within the limit: sent", r.writes.size() == 3);
        r.readBack(0, 15, 0, -1);   // firmware clamped
        check("read-back differs from the request: reported",
              r.bannerVisibleWith("not in force as typed") && r.hasText("in force: 15.0 deg"));
    }

    // 4. Level, explicit heading
    {
        Rig r;
        r.armed();
        r.spin(0)->setValue(12.0);
        r.button("Level")->click();
        check("Level writes 0 / 0 / -1",
              r.writes.size() == 3 && r.writes[0].second == 0.0 && r.writes[1].second == 0.0 && r.writes[2].second == -1.0);
        Rig q;
        q.armed();
        QCheckBox *hold = q.view.findChild<QCheckBox *>();
        hold->setChecked(false);
        q.spin(2)->setValue(270.0);
        q.button("Apply")->click();
        check("explicit heading 270 written as 270", q.writes.size() == 3 && q.writes[2].second == 270.0);
    }

    // 5. Drive: confirmation, read-back, firmware refusal
    {
        Rig r;
        r.armed();
        answerDialog(false);
        r.button("Drive")->click();
        check("Drive declined: nothing written", r.writes.isEmpty());
        answerDialog(true);
        r.button("Drive")->click();
        check("Drive confirmed: CAL_ctrlMode = 1 written", r.writes.size() == 1 && r.writes[0] == qMakePair(QString("CAL_ctrlMode"), 1.0));
        check("Drive pending: mode label says pending, not DRIVE", r.hasText("CAL_ctrlMode): pending"));
        r.readBack(1);
        check("fresh read-back 1: label DRIVE, Drive button checked", r.hasText("CAL_ctrlMode): DRIVE") && r.button("Drive")->isChecked());
    }
    {
        Rig r;
        r.armed();
        answerDialog(true);
        r.button("Drive")->click();
        r.readBack(0);   // firmware refused
        check("Drive refused by the firmware: reported, label stays SHADOW",
              r.bannerVisibleWith("Drive was not accepted") && r.hasText("CAL_ctrlMode): SHADOW"));
    }

    // 6. Shadow is always available with a link and writes mode 0
    {
        Rig r;
        r.armed();
        answerDialog(true);
        r.button("Drive")->click();
        r.readBack(1);
        r.writes.clear();
        check("driving: Shadow enabled", r.button("Shadow")->isEnabled());
        r.button("Shadow")->click();
        check("Shadow writes CAL_ctrlMode = 0", r.writes.size() == 1 && r.writes[0] == qMakePair(QString("CAL_ctrlMode"), 0.0));
        r.readBack(0);
        check("Shadow confirmed without a drop notice", r.hasText("CAL_ctrlMode): SHADOW") && !r.bannerVisibleWith("went back"));
    }

    // 7. manual throttle wins: Drive refused with the reason
    {
        Rig r;
        r.armed();
        QMap<QString, quint32> mv;
        mv["CAL_motorManual"] = 1;
        r.view.onMotorCharsRead(mv, true);
        check("manual ON: Drive disabled, reason shown", !r.button("Drive")->isEnabled() && r.hasText("Manual throttle is ON"));
    }

    // 8. trips and drops
    {
        Rig r;
        r.armed();
        answerDialog(true);
        r.button("Drive")->click();
        r.readBack(1);
        r.sample(2, 1, 0);
        r.sample(2, 1 | 2, 0);          // nav estimate invalid
        r.sample(2, 0, 1);              // trip counted, bits gone again
        check("trip: banner names the trip and the fault bit seen",
              r.bannerVisibleWith("dropped the controller to SHADOW") && r.bannerVisibleWith("navigation estimate invalid"));
        check("trip: counter shown", r.hasText("Trips since boot: 1"));
        r.readBack(0);
        check("trip: the mode read-back drop adds no second notice", !r.bannerVisibleWith("went back"));
    }
    {
        Rig r;
        r.armed();
        answerDialog(true);
        r.button("Drive")->click();
        r.readBack(1);
        r.sample(0);                    // DISARM
        check("leaving ARMED: read-back requested at once, entry disabled",
              r.readBacks > 0 && !r.button("Apply")->isEnabled());
        r.readBack(0);
        check("drop on disarm explained", r.bannerVisibleWith("left ARMED"));
    }

    // 9. stale link: entry disabled, numbers greyed, Shadow still there
    {
        Rig r;
        r.armed();
        r.sample(2, 0, 0, 3.5f, 3.5f);
        check("live: setpoint and actual shown", r.hasText("+3.50"));
        wait(2200);
        check("stale: entry disabled, STALE shown, numbers dashed",
              !r.button("Apply")->isEnabled() && r.hasText("STALE") && !r.hasText("+3.50"));
        check("stale: Shadow still reachable", r.button("Shadow")->isEnabled() || r.button("Shadow")->isChecked());
    }

    // 10. an unconfirmed write is given up, with a message
    {
        Rig r;
        r.armed();
        answerDialog(true);
        r.button("Drive")->click();
        wait(1700);
        r.sample(2);
        check("unconfirmed Drive: mode unknown + notice",
              r.hasText("CAL_ctrlMode): --") && r.bannerVisibleWith("not confirmed"));
    }

    // 11. an A2L without the controller: inert, says why
    {
        CtrlView v;
        v.setAvailable({});
        v.setCharsAvailable({});
        v.setConnected(true);
        bool any = false;
        for (QPushButton *b : v.findChildren<QPushButton *>())
            any = any || b->isEnabled();
        check("no controller in the A2L: every button disabled", !any);
    }

    // Optional: CTRL_VIEW_SHOT=<file.png> renders a driving panel with a few seconds of
    // synthetic data (layout inspection without a board).
    if (const QByteArray shot = qgetenv("CTRL_VIEW_SHOT"); !shot.isEmpty()) {
        Rig r;
        r.armed();
        r.view.resize(1100, 520);
        r.view.show();
        for (int i = 0; i < 60; ++i) {
            const float t = float(i) * 0.1f;
            r.sample(2, 1, 0, 8.0f * std::sin(t), 5.0f);
            wait(5);
        }
        r.readBack(1, 5, 0, -1, 12.0);
        r.view.grab().save(QString::fromLocal8Bit(shot));
    }

    std::printf("%s\n", g_fail == 0 ? "[ctrl_view_selfcheck] PASS" : "[ctrl_view_selfcheck] FAIL");
    return g_fail == 0 ? 0 : 1;
}
