cmake_minimum_required(VERSION 3.16)
project(rf_frontend_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets SerialPort)

set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/controlword.cpp
    src/cdccomm.cpp
    src/templatemanager.cpp
    src/themedata.cpp
    src/antennastatuspanel.cpp
    src/attenuatorpanel.cpp
    src/statspanel.cpp
)

set(HEADERS
    src/mainwindow.h
    src/controlword.h
    src/cdccomm.h
    src/templatemanager.h
    src/themedata.h
    src/antennastatuspanel.h
    src/attenuatorpanel.h
    src/statspanel.h
)

qt_add_executable(rf_frontend_control WIN32 ${SOURCES} ${HEADERS} src/app.rc)

target_link_options(rf_frontend_control PRIVATE "/MANIFEST:NO")

target_link_libraries(rf_frontend_control PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::SerialPort
)
