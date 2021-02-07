QT += core gui widgets

TARGET = DpSim
TEMPLATE = app
CONFIG += c++14
QT+=axcontainer
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    controller_interface.h \
    observer_interface.h \
    controller.h \
    observer.h \
    triopclib.h \
    servodriver.h \
    semicontrol.h \
    semicontrol_interface.h \
    position_interface.h \
    position_6dof.h

SOURCES += main.cpp \
    controller_interface.cpp \
    observer_interface.cpp \
    controller.cpp \
    observer.cpp \
    triopclib.cpp \
    servodriver.cpp \
    semicontrol.cpp \
    semicontrol_interface.cpp \
    position_interface.cpp \
    position_6dof.cpp

INCLUDEPATH+= C:\C++library\boost_1_65_1\boost_1_65_1
INCLUDEPATH+= C:\C++library\eigen3\eigen3\Eigen
INCLUDEPATH += C:\C++library\gsl\include

# LIBS += -L"C:/Program Files/GSL/lib" -lgsl -lgslcblas -lm
LIBS += -L C:\C++library\gsl\lib -llibgsl -llibgslcblas -lm

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpController/release/ -lDpController0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpController/debug/ -lDpController0

INCLUDEPATH += $$PWD/../DpController
DEPENDPATH += $$PWD/../DpController

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpObserver/release/ -lDpObserver0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpObserver/debug/ -lDpObserver0

INCLUDEPATH += $$PWD/../DpObserver
DEPENDPATH += $$PWD/../DpObserver

#win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpMsg/release/ -lDpMsg
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpMsg/debug/ -lDpMsg

#INCLUDEPATH += $$PWD/../DpMsg
#DEPENDPATH += $$PWD/../DpMsg

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpGui/release/ -lDpGui0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpGui/debug/ -lDpGui0

INCLUDEPATH += $$PWD/../DpGui
DEPENDPATH += $$PWD/../DpGui

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../position/release/ -lPosition0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../position/debug/ -lPosition0

INCLUDEPATH += $$PWD/../position
DEPENDPATH += $$PWD/../position



win32 {
    RC_ICONS = skloe.ico
    VERSION = 0.1.2.180911
    QMAKE_TARGET_COPYRIGHT = "Copyright (c) 2018, Bo Li"
}

RESOURCES += \
    dpsim_resource.qrc
