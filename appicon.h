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
inline const QColor Border(255, 255, 255);     // ring around the badge

inline QPixmap pixmap(int size)
{
    QPixmap pm(size, size);
    pm.fill(Qt::transparent);

    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setPen(Qt::NoPen);

    // White ring, so the badge stays visible against the red title bar.
    // Drawn as a filled rounded rect with the red one laid on top, rather
    // than as a stroke - a stroke straddles the path and would blur away at
    // 16px. Inset by half a pixel so the antialiased edge stays inside.
    const qreal ring = qMax(1.0, size * 0.08);
    const QRectF outer(0.5, 0.5, size - 1.0, size - 1.0);
    p.setBrush(Border);
    p.drawRoundedRect(outer, size * 0.18, size * 0.18);

    const QRectF inner = outer.adjusted(ring, ring, -ring, -ring);
    p.setBrush(Background);
    p.drawRoundedRect(inner, inner.width() * 0.16, inner.width() * 0.16);

    // Scale the font so "AUI" fills a fixed fraction of the badge at any
    // icon size: measure once at a guess, then correct by the ratio.
    const QString text = QStringLiteral("AUI");
    QFont f(QStringLiteral("Segoe UI"));
    f.setBold(true);
    f.setPixelSize(qMax(4, int(inner.width() * 0.5)));
    const qreal advance = QFontMetricsF(f).horizontalAdvance(text);
    if (advance > 0.0)
        f.setPixelSize(qMax(4, int(f.pixelSize() * (inner.width() * 0.82) / advance)));

    p.setFont(f);
    p.setPen(Foreground);
    p.drawText(inner, Qt::AlignCenter, text);
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
