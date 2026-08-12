#ifndef APPICON_H
#define APPICON_H

#include <QColor>
#include <QFont>
#include <QFontMetricsF>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QRectF>

// Application logo: the letters "AUI" on a rounded badge. Drawn at runtime
// instead of shipping an image file, so there is no resource to keep in sync
// and the icon stays sharp at every size Windows asks for.
namespace AppIcon {

inline const QColor Background(178, 34, 40);   // deep red badge
inline const QColor Foreground(240, 244, 248); // near-white letters

inline QPixmap pixmap(int size)
{
    QPixmap pm(size, size);
    pm.fill(Qt::transparent);

    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::TextAntialiasing);

    // Badge. Inset by half a pixel so the antialiased edge stays inside.
    const QRectF box(0.5, 0.5, size - 1.0, size - 1.0);
    p.setPen(Qt::NoPen);
    p.setBrush(Background);
    p.drawRoundedRect(box, size * 0.18, size * 0.18);

    // Scale the font so "AUI" fills a fixed fraction of the width at any
    // icon size: measure once at a guess, then correct by the ratio.
    const QString text = QStringLiteral("AUI");
    QFont f(QStringLiteral("Segoe UI"));
    f.setBold(true);
    f.setPixelSize(qMax(4, int(size * 0.5)));
    const qreal advance = QFontMetricsF(f).horizontalAdvance(text);
    if (advance > 0.0)
        f.setPixelSize(qMax(4, int(f.pixelSize() * (size * 0.76) / advance)));

    p.setFont(f);
    p.setPen(Foreground);
    p.drawText(QRectF(0, 0, size, size), Qt::AlignCenter, text);
    return pm;
}

// Several sizes in one QIcon: Windows picks 16px for the title bar and
// larger ones for the taskbar and Alt+Tab, and each is drawn to fit rather
// than scaled down from a single bitmap.
inline QIcon icon()
{
    QIcon ic;
    for (int size : {16, 20, 24, 32, 48, 64, 128, 256})
        ic.addPixmap(pixmap(size));
    return ic;
}

} // namespace AppIcon

#endif // APPICON_H
