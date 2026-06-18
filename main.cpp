#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    // QApplication haelt die Event-Loop am Laufen (wie in PyQt6).
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    // exec() blockiert bis das Fenster geschlossen wird.
    return app.exec();
}
