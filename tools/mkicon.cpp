// Writes aui.ico from the very same AppIcon painting code the running app
// uses. Explorer, the Start menu and a taskbar pin read the icon out of the
// exe's Win32 resource, which Qt's setWindowIcon() never touches - so the
// badge has to exist as a real file too.
//
// Not part of the normal build (it needs a Qt GUI session to render text).
// After changing appicon.h, regenerate the committed icon with:
//     cmake --build build --target aui-icon
#include "appicon.h"

#include <QBuffer>
#include <QFile>
#include <QGuiApplication>
#include <QImage>
#include <QtEndian>

namespace {

// The sizes Windows asks for: 16 in the title bar, 32/48 on the taskbar and
// in Explorer, 256 for the extra-large view. Each is painted at its target
// size instead of being scaled down from one bitmap.
constexpr int Sizes[] = {16, 20, 24, 32, 48, 64, 128, 256};

QByteArray pngFor(int size)
{
    QByteArray png;
    QBuffer buf(&png);
    buf.open(QIODevice::WriteOnly);
    AppIcon::pixmap(size).toImage().save(&buf, "PNG");
    return png;
}

void appendLE16(QByteArray &out, quint16 v)
{
    char b[2];
    qToLittleEndian(v, b);
    out.append(b, 2);
}

void appendLE32(QByteArray &out, quint32 v)
{
    char b[4];
    qToLittleEndian(v, b);
    out.append(b, 4);
}

} // namespace

// ICO container built by hand rather than through Qt's ico image plugin:
// the plugin is optional in a Qt build, and PNG-compressed entries (which
// Windows has accepted since Vista) keep the 256px frame small.
int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    if (argc < 2) {
        qWarning("usage: mkicon <output.ico>");
        return 2;
    }

    constexpr int count = int(sizeof(Sizes) / sizeof(Sizes[0]));
    QVector<QByteArray> frames;
    for (int size : Sizes)
        frames.append(pngFor(size));

    QByteArray dir;
    appendLE16(dir, 0);              // reserved
    appendLE16(dir, 1);              // type: icon
    appendLE16(dir, quint16(count));

    // Image data starts after the header and the fixed-size directory.
    quint32 offset = quint32(6 + 16 * count);
    for (int i = 0; i < count; ++i) {
        const int size = Sizes[i];
        dir.append(char(size == 256 ? 0 : size));   // 0 means 256
        dir.append(char(size == 256 ? 0 : size));
        dir.append(char(0));         // palette entries: none, it is 32-bit
        dir.append(char(0));         // reserved
        appendLE16(dir, 1);          // colour planes
        appendLE16(dir, 32);         // bits per pixel
        appendLE32(dir, quint32(frames[i].size()));
        appendLE32(dir, offset);
        offset += quint32(frames[i].size());
    }

    QFile out(QString::fromLocal8Bit(argv[1]));
    if (!out.open(QIODevice::WriteOnly)) {
        qWarning("cannot write %s: %s", argv[1],
                 qPrintable(out.errorString()));
        return 1;
    }
    out.write(dir);
    for (const QByteArray &frame : frames)
        out.write(frame);
    out.close();
    return 0;
}
