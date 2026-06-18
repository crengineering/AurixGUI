# AURIX Serial Monitor (Phase 1)

Minimaler serieller Monitor in C++/Qt6, der PuTTY funktional ersetzt:
Port auswaehlen, verbinden, eingehende Daten anzeigen. Nur lesen.

## Voraussetzungen (Windows)
- Qt 6 (Online Installer), Kit: **MinGW** (bringt Compiler + Ninja mit)
- CMake (im Qt-Installer als Tool enthalten)

## Build (MinGW, ueber die CLI)
Pfade ggf. an die eigene Qt-Installation anpassen:

```bat
set QT=C:\Qt\6.11.1\mingw_64
set PATH=C:\Qt\Tools\mingw1310_64\bin;C:\Qt\Tools\Ninja;%PATH%

cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=%QT%
cmake --build build
```

Start:
```bat
build\SerialMonitor.exe
```

Falls beim Start Qt-DLLs fehlen, einmalig:
```bat
%QT%\bin\windeployqt.exe build\SerialMonitor.exe
```

## Bekannte Stolpersteine
- Der Port wird **exklusiv** geoeffnet. PuTTY und dieses Tool koennen NICHT
  gleichzeitig denselben COM-Port offen haben. PuTTY vorher schliessen.
- Bei MSVC-Kit statt MinGW: UTF-8-Quelltext mit `/utf-8` kompilieren
  (in CMakeLists `target_compile_options(... /utf-8)`), sonst falsche Umlaute.

## Naechster Schritt (Phase 2: Senden)
1. In `toggleConnection()` `QIODevice::ReadOnly` -> `QIODevice::ReadWrite`.
2. Ein `QLineEdit` + Button "Senden" ergaenzen.
3. Im Slot: `m_serial->write(line.toUtf8() + "\r\n");`
