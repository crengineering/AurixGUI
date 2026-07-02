#ifndef LAMPICON_H
#define LAMPICON_H

#include <QColor>
#include <QIcon>
#include <QPainter>
#include <QPixmap>

// Round status lamp used in tab bars and panels:
// green = connected / all good, red = disconnected / error, gray = unknown.
inline QPixmap lampPixmap(const QColor &color, int size = 12)
{
    QPixmap pm(size, size);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(color);
    p.setPen(QPen(color.darker(140), 1));
    p.drawEllipse(1, 1, size - 2, size - 2);
    return pm;
}

inline QIcon lampIcon(const QColor &color)
{
    return QIcon(lampPixmap(color));
}

namespace LampColor {
inline const QColor Green(60, 170, 60);
inline const QColor Red(200, 50, 50);
inline const QColor Gray(140, 140, 140);
}

#endif // LAMPICON_H
