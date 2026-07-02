#include <QApplication>
#include <QString>
#include <cmath>
#include <cstdio>
#include "mainwindow.h"
#include "mf4writer.h"

// Headless self-test: writes a synthetic MF4 file so the writer can be
// validated (e.g. with asammdf) without clicking through the GUI.
static int selftestMf4(const QString &path)
{
    Mf4Writer w;
    for (int i = 0; i < 100; ++i) {
        Mf4Writer::Sample s;
        s.t     = i * 0.1;
        s.dts   = 50.0f + 2.0f * float(std::sin(i * 0.1));
        s.dtsc  = 50.5f + 2.0f * float(std::sin(i * 0.1 + 0.2));
        s.vdd   = 1.25f;
        s.vddp3 = 3.30f;
        s.vext  = 5.00f;
        s.tick  = quint32(1000 + i * 100);
        s.diag  = (i >= 50) ? 0x10u : 0x00u;
        w.append(s);
    }
    QString err;
    if (!w.save(path, &err)) {
        std::fprintf(stderr, "MF4 selftest failed: %s\n", qPrintable(err));
        return 1;
    }
    std::printf("MF4 selftest written: %s (%d samples)\n", qPrintable(path), w.count());
    return 0;
}

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc - 1; ++i) {
        if (QString::fromLocal8Bit(argv[i]) == "--selftest-mf4")
            return selftestMf4(QString::fromLocal8Bit(argv[i + 1]));
    }

    // QApplication haelt die Event-Loop am Laufen (wie in PyQt6).
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    // exec() blockiert bis das Fenster geschlossen wird.
    return app.exec();
}
