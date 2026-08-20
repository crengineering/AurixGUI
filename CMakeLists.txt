cmake_minimum_required(VERSION 3.16)

project(SerialMonitor VERSION 0.1 LANGUAGES CXX)

# Windows resources (the app icon) are compiled by windres/rc, which needs
# its own language enabled - project() above only asks for CXX.
if(WIN32)
    enable_language(RC)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# AUTOMOC laesst CMake den Meta-Object-Compiler (moc) automatisch ueber
# Header mit Q_OBJECT laufen. Ohne das funktionieren Signals/Slots nicht.
set(CMAKE_AUTOMOC ON)

# Sucht die Qt6-Komponenten. SerialPort ist das Modul fuer QSerialPort,
# Concurrent fuer den Port-Scan im Hintergrund-Thread.
find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort Network Concurrent)

add_executable(SerialMonitor
    main.cpp
    mainwindow.cpp
    mainwindow.h
    lampicon.h
    appicon.h
    titlebar.h
    xcpclient.cpp
    xcpclient.h
    xcppanel.cpp
    xcppanel.h
    a2lmodel.cpp
    a2lmodel.h
    plotwidget.cpp
    plotwidget.h
    mf4writer.cpp
    mf4writer.h
    plotpane.cpp
    plotpane.h
    systemfooter.cpp
    systemfooter.h
    $<$<BOOL:${WIN32}>:app.rc>
)

# windres resolves the ICON path against its include dirs, so point it at the
# source dir where aui.ico lives.
target_include_directories(SerialMonitor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Regenerates the committed aui.ico from appicon.h. Deliberately not part of
# ALL: it renders text and therefore needs Qt's GUI stack at build time, which
# a plain build should not depend on. Run it by hand after changing the badge:
#     cmake --build build --target aui-icon
if(WIN32)
    add_executable(mkicon EXCLUDE_FROM_ALL tools/mkicon.cpp appicon.h)
    target_include_directories(mkicon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(mkicon PRIVATE Qt6::Gui)

    add_custom_target(aui-icon
        COMMAND mkicon ${CMAKE_CURRENT_SOURCE_DIR}/aui.ico
        DEPENDS mkicon
        COMMENT "Rendering aui.ico from appicon.h")
endif()

# Baut die exe als GUI-Anwendung (Windows-Subsystem) statt als Konsolen-
# Programm. Ohne das oeffnet Windows beim Start zusaetzlich ein Terminal.
# Qt liefert dafuer ueber Qt6::EntryPointPrivate das passende WinMain.
set_target_properties(SerialMonitor PROPERTIES WIN32_EXECUTABLE ON)

target_link_libraries(SerialMonitor PRIVATE
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::Network
    Qt6::Concurrent
)

# dwmapi liefert DwmSetWindowAttribute fuer die eingefaerbte Titelleiste.
if(WIN32)
    target_link_libraries(SerialMonitor PRIVATE dwmapi)
endif()
