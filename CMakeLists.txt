cmake_minimum_required(VERSION 3.16)

project(SerialMonitor VERSION 0.1 LANGUAGES CXX)

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
)

target_link_libraries(SerialMonitor PRIVATE
    Qt6::Widgets
    Qt6::SerialPort
    Qt6::Network
    Qt6::Concurrent
)
