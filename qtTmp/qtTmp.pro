TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    costfunction.cpp \
    dynamicmodel.cpp \
    ilqrsolver.cpp \
    romeosimpleactuator.cpp \
    costfunctionromeoactuator.cpp \
    romeolinearactuator.cpp \
    mainMPC.cpp

include(deployment.pri)
qtcAddDeployment()

DISTFILES += \
    qtTmp.pro.user

HEADERS += \
    costfunction.h \
    dynamicmodel.h \
    ilqrsolver.h \
    romeosimpleactuator.h \
    costfunctionromeoactuator.h \
    config.h \
    romeolinearactuator.h

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/python2.7
