#ifndef LAMPICON_H
#define LAMPICON_H

#include <QColor>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QRadialGradient>

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

// Bulb variant for the system footer: the same bead, plus a soft halo when it
// is lit. The halo is needed because the footer strip is red, which swallows a
// bare bead of this size.
//
// The bead is always `bead` px across and centred in a (bead + 2*margin)
// pixmap whatever the state, so a lamp switching on or off changes only its
// brightness - never its size, and never the position of the label next to it.
inline QPixmap bulbPixmap(const QColor &color, bool lit,
                          int bead = 12, int margin = 4)
{
    const int size = bead + 2 * margin;
    QPixmap pm(size, size);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);

    if (lit) {
        // Hold the glow at full strength out to the rim of the bead, then fade
        // it across the margin: a gradient that starts fading at the centre
        // just looks like a blurred bead.
        QRadialGradient halo(size / 2.0, size / 2.0, size / 2.0);
        QColor c = color;
        c.setAlpha(150); halo.setColorAt(bead / double(size) * 0.5, c);
        c.setAlpha(0);   halo.setColorAt(1.0, c);
        p.setPen(Qt::NoPen);
        p.setBrush(halo);
        p.drawEllipse(QRectF(0, 0, size, size));
    }

    p.setBrush(color);
    p.setPen(QPen(color.darker(140), 1));
    p.drawEllipse(QRectF(margin, margin, bead, bead));
    return pm;
}

namespace LampColor {
inline const QColor Green(60, 170, 60);
inline const QColor Red(200, 50, 50);
inline const QColor Gray(140, 140, 140);
// An unlit bulb: the part was probed and did not answer. Deliberately not red -
// the footer strip itself is red, so a red bead there is nearly invisible.
inline const QColor Off(58, 58, 58);
}

#endif // LAMPICON_H
