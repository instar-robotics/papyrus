#include "diagramview.h"
#include "constants.h"
#include "librarypanel.h"

#include <iostream>
#include <QWheelEvent>
#include <QMimeData>
#include <QTreeWidgetItem>

DiagramView::DiagramView(QWidget *parent) : QGraphicsView(parent)
{
    // Make it so that transformations (essentially zooming) are centered on mouse
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setMouseTracking(true);
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    setDragMode(QGraphicsView::RubberBandDrag);
}

DiagramView::DiagramView(QGraphicsScene *scene, QWidget *parent) : DiagramView(parent)
{
    setScene(scene);
}

void DiagramView::wheelEvent(QWheelEvent *evt)
{
    // Zoom if CTRL is pressed while scrolling
    if (evt->modifiers() & Qt::ControlModifier) {

        // Handle direction of zoom
        if (evt->delta() > 0)
            scale(SCALE_FACTOR, SCALE_FACTOR);
        else
            scale(1 / SCALE_FACTOR, 1 / SCALE_FACTOR);
    } else {
        // If CTRL is not pressed, simply scroll
        QGraphicsView::wheelEvent(evt);
    }
}

/*
void DiagramView::dragEnterEvent(QDragEnterEvent *evt)
{
    std::cout << "Drag enter in view" << std::endl;
    evt->accept(); // Allows the 'dragMoveEvent' to happen
//    QGraphicsView::dragEnterEvent(evt);
}
//*/

/*
void DiagramView::dragLeaveEvent(QDragLeaveEvent *evt)
{
    std::cout << "Drag leave in view" << std::endl;
//    QGraphicsView::dragLeaveEvent(evt);
}
//*/

/*
void DiagramView::dragMoveEvent(QDragMoveEvent *evt)
{
    std::cout << "Drag move event in view" << std::endl;
    evt->accept(); // Allows the 'dropEvent' to happen (seems to work anyway)
//    QGraphicsView::dragMoveEvent(evt);
}
//*/

/*
void DiagramView::dropEvent(QDropEvent *evt)
{
    QGraphicsView::dropEvent(evt);
    return;
    // Filter the drop event
    if (evt->mimeData()->hasFormat(LibraryPanel::libraryItemMimeType())) {
        // If this is a drop from the library
        QByteArray pieceData = evt->mimeData()->data(LibraryPanel::libraryItemMimeType());
        QDataStream dataStream(&pieceData, QIODevice::ReadOnly);
        QString name;

        dataStream >> name;

//        scene()->addText()

        evt->setDropAction(Qt::CopyAction);
        evt->accept();
    } else {
        // If the mime data format is unknown, ignore the drop event
        // TODO: emit a "showInformation" signal and connet to it from the main window to display in status bar
        evt->ignore();
    }
}
//*/
