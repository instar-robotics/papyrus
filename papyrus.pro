######################################################################
# Automatically generated by qmake (3.1) Mon Jul 23 18:41:40 2018
######################################################################

QT += core gui xml svg widgets
TEMPLATE = app
TARGET = papyrus
INCLUDEPATH += . include/

# The following define makes your compiler warn you if you use any
# feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Input
HEADERS += include/category.h \
           include/constants.h \
           include/diagrambox.h \
           include/diagramscene.h \
           include/diagramview.h \
           include/function.h \
           include/helpers.h \
           include/homepage.h \
           include/inputslot.h \
           include/library.h \
           include/librarypanel.h \
           include/link.h \
           include/outputslot.h \
           include/papyruswindow.h \
           include/propertiespanel.h \
           include/rosnode.h \
           include/rossession.h \
           include/script.h \
           include/slot.h \
           include/xmldescriptionreader.h \
           include/xmlscriptreader.h \
           include/nodeschooser.h \
           include/types.h \
           include/messagehandler.h \
           include/constantdiagrambox.h \
           include/constantfunction.h \
           include/datavisualization.h \
           include/datafetcher.h \
           include/scalarvisualization.h \
           include/scalarfetcher.h \
           include/matrixfetcher.h \
           include/matrixvisualization.h \
           include/zone.h \
           include/setcolorbutton.h \
           include/changelog.h \
           include/proplineedit.h \
           include/propdoublespinbox.h \
           include/movecommand.h \
           include/addboxcommand.h \
           include/swapboxescommand.h \
           include/addlinkcommand.h \
           include/addzonecommand.h \
           include/updateboxcommand.h \
           include/updatelinkcommand.h \
           include/updatezonecommand.h \
           include/udpatescriptcommand.h \
           include/activityfetcher.h \
           include/activityvisualizer.h \
           include/activityvisualizerbars.h \
           include/activityvisualizerthermal.h \
           include/inhibinput.h \
          include/mathtransfo.h \
          include/deletelinkcommand.h \
          include/deleteboxcommand.h \
          include/deletezonecommand.h \
          include/camera.h \
          include/light.h \
          include/shaderwidget.h \
          include/shadermatrix.h \
          include/shadersurface.h \
          include/shaderproxy.h \
          include/shadersurfacealpha.h \
          include/linkelement.h \
          include/finddialog.h \
    include/shadermovebar.h \
    include/shaderbarchart.h \
    include/shaderwireframe.h \
    include/shaderscaleplanes.h \
    include/shaderaxesplanes.h \
    include/scopewindow.h \
    include/scopemessage.h \
    include/scopeitem.h \
    include/shadercircular.h \
    include/shadercrown.h \
    include/shaderscalecircular.h \
    include/shaderarrow.h \
    include/shaderadds.h \
    include/rttokenmessage.h \
    include/shaderscalecylinder.h \
    include/shaderbarcircle.h \
    include/shaderpolar.h \
    include/shaderbarpolar.h \
    include/shaderwireframepolar.h \
    include/shadersurfacepolar.h \
    include/shaderscalepolar.h \
    include/shaderscale.h \
    include/threadshader.h \
    include/shadercompass.h \
    include/shaderscaleallplanes.h \
    include/shaderconechart.h \
    include/linkvisutobox.h \
    include/visufunctions.h \
    include/visuparamdialog.h \
    include/circularvisuparamdialog.h \
    include/polarvisuparamdialog.h

FORMS += src/connectivitywindow.ui src/papyruswindow.ui \
    src/nodeschooser.ui \
    src/nodeschooser.ui \
    src/scopewindow.ui

SOURCES += src/category.cpp \
           src/diagrambox.cpp \
           src/diagramscene.cpp \
           src/diagramview.cpp \
           src/function.cpp \
           src/helpers.cpp \
           src/homepage.cpp \
           src/inputslot.cpp \
           src/library.cpp \
           src/librarypanel.cpp \
           src/link.cpp \
           src/main.cpp \
           src/outputslot.cpp \
           src/papyruswindow.cpp \
           src/propertiespanel.cpp \
           src/rosnode.cpp \
           src/rossession.cpp \
           src/script.cpp \
           src/slot.cpp \
           src/xmldescriptionreader.cpp \
           src/xmlscriptreader.cpp \
           src/nodeschooser.cpp \
           src/messagehandler.cpp \
           src/constantdiagrambox.cpp \
           src/constantfunction.cpp \
           src/datavisualization.cpp \
           src/datafetcher.cpp \
           src/scalarvisualization.cpp \
           src/scalarfetcher.cpp \
           src/matrixfetcher.cpp \
           src/matrixvisualization.cpp \
           src/zone.cpp \
           src/setcolorbutton.cpp \
           src/proplineedit.cpp \
           src/propdoublespinbox.cpp \
           src/movecommand.cpp \
           src/addboxcommand.cpp \
           src/swapboxescommand.cpp \
           src/addlinkcommand.cpp \
           src/addzonecommand.cpp \
           src/updateboxcommand.cpp \
           src/updatelinkcommand.cpp \
           src/updatezonecommand.cpp \
           src/updatescriptcommand.cpp \
           src/activityfetcher.cpp \
           src/activityvisualizer.cpp \
           src/activityvisualizerbars.cpp \
           src/activityvisualizerthermal.cpp \
           src/inhibinput.cpp \
           src/camera.cpp \
           src/light.cpp \
           src/shaderwidget.cpp \
           src/shadermatrix.cpp \
           src/shadersurface.cpp \
           src/mathtransfo.cpp \
           src/deletelinkcommand.cpp \
           src/deleteboxcommand.cpp \
           src/deletezonecommand.cpp \
        src/shaderproxy.cpp \
        src/linkelement.cpp \
        src/finddialog.cpp \
    src/shadermovebar.cpp \
    src/shaderbarchart.cpp \
    src/shaderconechart.cpp \
    src/shaderwireframe.cpp \
    src/shaderscaleplanes.cpp \
    src/shaderaxesplanes.cpp \
    src/scopewindow.cpp \
    src/scopemessage.cpp \
    src/scopeitem.cpp \
    src/shadercircular.cpp \
    src/shadercrown.cpp \
    src/shaderscalecircular.cpp \
    src/shaderarrow.cpp \
    src/shaderadds.cpp \
    src/rttokenmessage.cpp \
    src/shadercircular.cpp \
    src/shaderscalecylinder.cpp \
    src/shaderbarcircle.cpp \
    src/shaderpolar.cpp \
    src/shaderbarpolar.cpp \
    src/shaderwireframepolar.cpp \
    src/shadersurfacepolar.cpp \
    src/shaderscalepolar.cpp \
    src/shaderscale.cpp \
    src/threadshader.cpp \
    src/shadercompass.cpp \
    src/shaderscaleallplanes.cpp \
    src/shaderconechart.cpp \
    src/linkvisutobox.cpp \
    src/visufunctions.cpp \
    src/visuparamdialog.cpp \
    src/circularvisuparamdialog.cpp \
    src/polarvisuparamdialog.cpp

RESOURCES += icons.qrc \
    shaders.qrc

DISTFILES +=
