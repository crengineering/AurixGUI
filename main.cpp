#include <QApplication>
#include <QLocalServer>
#include <QLocalSocket>
#include <QString>
#include <cmath>
#include <cstdio>
#include "appicon.h"
#include "mainwindow.h"
#include "mf4writer.h"

// Headless self-test: writes a synthetic MF4 file so the writer can be
// validated (e.g. with asammdf) without clicking through the GUI.
static int selftestMf4(const QString &path)
{
    Mf4Writer w;
    w.begin({
        {"DieTemp_DTS",     "\xC2\xB0""C", true},
        {"DieTemp_DTSC",    "\xC2\xB0""C", true},
        {"VDD",             "V",           true},
        {"VDDP3",           "V",           true},
        {"VEXT",            "V",           true},
        {"BaroPressure",    "hPa",         true},
        {"BaroTemperature", "\xC2\xB0""C", true},
        {"BaroAltitude",    "m",           true},
        {"TickMs",          "ms",          false},
        {"DiagStatus",      QString(),     false},
    });
    for (int i = 0; i < 100; ++i) {
        const double p = 956.0 + 0.5 * std::sin(i * 0.1);
        w.append(i * 0.1, {
            50.0 + 2.0 * std::sin(i * 0.1),
            50.5 + 2.0 * std::sin(i * 0.1 + 0.2),
            1.25, 3.30, 5.00,
            p,
            26.8,
            44330.0 * (1.0 - std::pow(p / 1013.25, 0.190295)),
            double(1000 + i * 100),
            double((i >= 50) ? 0x10u : 0x00u),
        });
    }
    QString err;
    if (!w.save(path, &err)) {
        std::fprintf(stderr, "MF4 selftest failed: %s\n", qPrintable(err));
        return 1;
    }
    std::printf("MF4 selftest written: %s (%d samples)\n", qPrintable(path), w.count());
    return 0;
}

// --- single instance -------------------------------------------------------
//
// Two copies of this application fight over the same serial port. The loser
// gets a silent "access denied" and shows no data, which looks like a dead
// board rather than a second window. That cost real debugging time on
// 2026-08-25: an instance left running for 39 minutes held COM4 while a fresh
// one sat there showing nothing.
//
// A named local socket rather than QSharedMemory, for two reasons: the OS
// reclaims it when the process dies, and it lets the second launch hand the
// first one a nudge -- so double-clicking the icon again raises the existing
// window, which is what the user meant by it.
static const char *const kSingletonName = "AurixMonitor.singleton";

// Returns true if another instance answered (and was asked to come forward).
static bool raiseRunningInstance()
{
    QLocalSocket probe;
    probe.connectToServer(QString::fromLatin1(kSingletonName));
    if (!probe.waitForConnected(300))
        return false;                   // nobody listening: we are the first
    probe.write("raise");
    probe.waitForBytesWritten(300);
    probe.disconnectFromServer();
    return true;
}

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc - 1; ++i) {
        if (QString::fromLocal8Bit(argv[i]) == "--selftest-mf4")
            return selftestMf4(QString::fromLocal8Bit(argv[i + 1]));
    }

    // QApplication haelt die Event-Loop am Laufen (wie in PyQt6).
    QApplication app(argc, argv);

    // Deterministic QSettings location. Without an explicit organisation and
    // application name Qt derives one from the executable, so the remembered
    // A2L path and the saved plot layout could land somewhere unexpected -
    // or not be found again after a rename.
    QCoreApplication::setOrganizationName("AurixTricore");
    QCoreApplication::setApplicationName("AurixMonitor");

    // Gilt fuer alle Fenster und Dialoge - Titelleiste, Taskleiste, Alt+Tab.
    QApplication::setWindowIcon(AppIcon::icon());

    if (raiseRunningInstance()) {
        std::printf("AurixMonitor is already running - raising that window.\n");
        return 0;
    }

    // A hard kill can leave the name behind on some platforms; removeServer()
    // is a no-op when nothing stale is there. If listen() still fails we carry
    // on WITHOUT the guard rather than refusing to start - a missing guard is
    // an annoyance, a GUI that will not launch is worse.
    QLocalServer::removeServer(QString::fromLatin1(kSingletonName));
    QLocalServer singleton;
    if (!singleton.listen(QString::fromLatin1(kSingletonName))) {
        std::fprintf(stderr, "single-instance guard inactive: %s\n",
                     qPrintable(singleton.errorString()));
    }

    MainWindow window;
    window.show();

    // A later launch connects, says nothing useful, and exits; all we owe it is
    // to bring this window to the front.
    QObject::connect(&singleton, &QLocalServer::newConnection, &window, [&]() {
        if (QLocalSocket *c = singleton.nextPendingConnection())
            c->deleteLater();
        window.setWindowState(window.windowState() & ~Qt::WindowMinimized);
        window.show();
        window.raise();
        window.activateWindow();
    });

    // exec() blockiert bis das Fenster geschlossen wird.
    return app.exec();
}
