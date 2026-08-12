#ifndef TITLEBAR_H
#define TITLEBAR_H

#include <QColor>
#include <QWidget>

#ifdef Q_OS_WIN
#include <dwmapi.h>
#include <windows.h>
#endif

// Colours the native Windows title bar. Uses DWM rather than a frameless
// window with a hand-drawn caption, which would mean reimplementing dragging,
// snapping and the system menu.
namespace TitleBar {

#ifdef Q_OS_WIN
// Defined by the Windows 11 SDK; spelled out here because the MinGW headers
// are not guaranteed to carry them.
constexpr DWORD AttrCaptionColor = 35;  // DWMWA_CAPTION_COLOR
constexpr DWORD AttrTextColor    = 36;  // DWMWA_TEXT_COLOR
#endif

// Needs Windows 11 (build 22000+). Older Windows rejects the attributes and
// keeps its default caption, which is why the results are not checked.
inline void applyColors(QWidget *w, const QColor &caption, const QColor &text)
{
#ifdef Q_OS_WIN
    // winId() forces creation of the native handle the call needs.
    const HWND hwnd = reinterpret_cast<HWND>(w->winId());
    if (!hwnd)
        return;

    // COLORREF is 0x00BBGGRR - byte order reversed from QColor::rgb().
    const COLORREF captionRef = RGB(caption.red(), caption.green(), caption.blue());
    const COLORREF textRef    = RGB(text.red(), text.green(), text.blue());
    DwmSetWindowAttribute(hwnd, AttrCaptionColor, &captionRef, sizeof(captionRef));
    DwmSetWindowAttribute(hwnd, AttrTextColor, &textRef, sizeof(textRef));
#else
    Q_UNUSED(w);
    Q_UNUSED(caption);
    Q_UNUSED(text);
#endif
}

} // namespace TitleBar

#endif // TITLEBAR_H
