TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += main.cpp \
    ilqrsolver.cpp \
    dynamicmodel.cpp \
    costfunction.cpp \
    romeosimpleactuator.cpp

HEADERS += \
    ilqrsolver.h \
    dynamicmodel.h \
    costfunction.h \
    romeosimpleactuator.h

INCLUDEPATH += /usr/include/eigen3
