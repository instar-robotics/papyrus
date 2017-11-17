#-------------------------------------------------
#
# Project created by QtCreator 2017-11-15T14:22:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Papyrus
TEMPLATE = app


SOURCES += main.cpp\
        papyruswindow.cpp \
    diagramscene.cpp \
    diagramview.cpp

HEADERS  += papyruswindow.h \
    constants.h \
    diagramscene.h \
    diagramview.h

FORMS    += papyruswindow.ui

RESOURCES += \
    icons.qrc
