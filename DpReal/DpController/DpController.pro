#-------------------------------------------------
#
# Project created by QtCreator 2018-04-29T15:39:15
#
#-------------------------------------------------

QT       -= gui

TARGET = DpController
TEMPLATE = lib

DEFINES += DPCONTROLLER_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    qpOASES/extras/OQPinterface.hpp \
    qpOASES/extras/SolutionAnalysis.hpp \
    qpOASES/Bounds.hpp \
    qpOASES/Constants.hpp \
    qpOASES/ConstraintProduct.hpp \
    qpOASES/Constraints.hpp \
    qpOASES/Flipper.hpp \
    qpOASES/Indexlist.hpp \
    qpOASES/LapackBlasReplacement.hpp \
    qpOASES/Matrices.hpp \
    qpOASES/MessageHandling.hpp \
    qpOASES/Options.hpp \
    qpOASES/QProblem.hpp \
    qpOASES/QProblemB.hpp \
    qpOASES/SparseSolver.hpp \
    qpOASES/SQProblem.hpp \
    qpOASES/SQProblemSchur.hpp \
    qpOASES/SubjectTo.hpp \
    qpOASES/Types.hpp \
    qpOASES/UnitTesting.hpp \
    qpOASES/Utils.hpp \
    backstepping_controller.h \
    basic_controller.h \
    basic_thrust_allocation.h \
    dpcontroller_global.h \
    fuzzy_controller.h \
    pid_controller.h \
    pm_controller.h \
    qpOASES.hpp \
    sqp_thrust_allocation.h \
    inverse_thrust_allocation.h

SOURCES += \
    backstepping_controller.cpp \
    basic_controller.cpp \
    basic_thrust_allocation.cpp \
    pid_controller.cpp \
    pm_controller.cpp \
    sqp_thrust_allocation.cpp \
    inverse_thrust_allocation.cpp \
    fuzzy_controller.cpp


INCLUDEPATH += $$PWD/include
INCLUDEPATH += C:\C++library\eigen3\eigen3\Eigen

LIBS += -L$$PWD/lib -lqpOASES

PRE_TARGETDEPS += $$PWD/lib/libqpOASES.a

win32 {
    VERSION = 0.1.0.180911
    QMAKE_TARGET_COPYRIGHT = "Copyright (c) 2018, Bo Li"
}
