#QT       += core gui
QT -= gui
TARGET = Position
TEMPLATE = lib

DEFINES += POSITION_LIBRARY
#QT += axcontainer
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += console c++11
#CONFIG -= app_bundle
#CONFIG += qt
#LIBS += E:\Qt\Qt5.10.0\Tools\mingw530_32\i686-w64-mingw32\lib -lws2_32
DEFINES += QT_DEPRECATED_WARNINGS
LIBS += -lws2_32
LIBS += -liphlpapi
SOURCES += \
    Markup.cpp \
    Network.cpp \
    RTPacket.cpp \
    RTProtocol.cpp \
    position.cpp \
#    main.cpp

#DISTFILES +=

HEADERS += \
    Markup.h \
    Network.h \
    RTPacket.h \
    RTProtocol.h \
    position.h \
    position_global.h

#win32:CONFIG(release, debug|release): LIBS += -LC:/Qwt-6.1.3/lib/ -lqwt
#else:win32:CONFIG(debug, debug|release): LIBS += -LC:/Qwt-6.1.3/lib/ -lqwtd

#INCLUDEPATH += C:/Qwt-6.1.3/include
#DEPENDPATH += C:/Qwt-6.1.3/include
win32 {
    VERSION = 0.1.0.180904
    QMAKE_TARGET_COPYRIGHT = "Copyright (c) 2018, Bo Li"
}
