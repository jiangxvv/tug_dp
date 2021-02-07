#-------------------------------------------------
#
# Project created by QtCreator 2018-05-23T22:06:12
#
#-------------------------------------------------

QT       += widgets

TARGET = DpGui
TEMPLATE = lib

DEFINES += DPGUI_LIBRARY

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
    core_window.cpp \
#    demo.cpp \
#    demo_interface.cpp \
#    demo_node.cpp \
    polar_plot.cpp \
    trajectory.cpp

HEADERS += \
    dpgui_global.h \
    core_window.h \
#    demo.h \
#    demo_interface.h \
    polar_plot.h \
    trajectory.h \
#    ui_demo.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

#FORMS += \
#    demo.ui

win32 {
    VERSION = 0.1.1.180914
    QMAKE_TARGET_COPYRIGHT = "Copyright (c) 2018, Bo Li"
}
