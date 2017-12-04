#-------------------------------------------------
#
# Project created by QtCreator 2017-11-15T14:22:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4) {
    QT += widgets
    CONFIG += c++11
} else {
    QMAKE_CXXFLAGS += -std=c++11
}

TARGET = Papyrus
TEMPLATE = app

LIBS += -L/nix/store/yk8k2m6zq1ih0dr46j86fmh3710cp6a7-qtbase-5.9.1/lib

SOURCES += main.cpp\
        papyruswindow.cpp \
    diagramscene.cpp \
    diagramview.cpp \
    diagrambox.cpp \
    arrow.cpp \
    librarypanel.cpp

HEADERS  += papyruswindow.h \
    constants.h \
    diagramscene.h \
    diagramview.h \
    diagrambox.h \
    arrow.h \
    librarypanel.h

FORMS    += papyruswindow.ui

RESOURCES += \
    icons.qrc
