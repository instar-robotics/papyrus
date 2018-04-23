#-------------------------------------------------
#
# Project created by QtCreator 2017-11-15T14:22:58
#
#-------------------------------------------------

QT       += core gui xml

greaterThan(QT_MAJOR_VERSION, 4) {
    QT += widgets svg
    CONFIG += c++11
} else {
    QMAKE_CXXFLAGS += -std=c++11
}

QMAKE_CXXFLAGS += -O2

TARGET = Papyrus
TEMPLATE = app

#LIBS += -L/nix/store/yk8k2m6zq1ih0dr46j86fmh3710cp6a7-qtbase-5.9.1/lib

LIBS += -L/nix/store/8d54pnh1wqv12ghwqq9psv7r9sdzbkvq-qtbase-5.9.1/lib -lcryptopp

SOURCES += main.cpp\
        papyruswindow.cpp \
    diagramscene.cpp \
    diagramview.cpp \
    diagrambox.cpp \
    arrow.cpp \
    librarypanel.cpp \
    library.cpp \
    category.cpp \
    function.cpp \
    xmldescriptionreader.cpp \
    script.cpp \
    xmlscriptreader.cpp \
    slot.cpp \
    inputslot.cpp \
    outputslot.cpp \
    propertiespanel.cpp \
    helpers.cpp \
    link.cpp

HEADERS  += papyruswindow.h \
    constants.h \
    diagramscene.h \
    diagramview.h \
    diagrambox.h \
    arrow.h \
    librarypanel.h \
    library.h \
    category.h \
    function.h \
    types.h \
    xmldescriptionreader.h \
    script.h \
    xmlscriptreader.h \
    slot.h \
    inputslot.h \
    outputslot.h \
    propertiespanel.h \
    helpers.h \
    link.h

FORMS    += papyruswindow.ui

RESOURCES += \
    icons.qrc
