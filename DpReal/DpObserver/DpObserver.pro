#-------------------------------------------------
#
# Project created by QtCreator 2018-04-30T22:59:16
#
#-------------------------------------------------

QT       -= gui

TARGET = DpObserver
TEMPLATE = lib

DEFINES += DPOBSERVER_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        passive_nonlinear_observer.cpp \
    alpha_belta_filter.cpp

HEADERS += \
        passive_nonlinear_observer.h \
        dpobserver_global.h \ 
    alpha_belta_filter.h

INCLUDEPATH+= C:\C++library\boost_1_65_1\boost_1_65_1
INCLUDEPATH+= C:\C++library\eigen3\eigen3\Eigen

win32 {
    VERSION = 0.1.0.180911
    QMAKE_TARGET_COPYRIGHT = "Copyright (c) 2018, Bo Li"
}
