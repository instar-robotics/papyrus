#include "diagramview.h"
#include "constants.h"

#include <iostream>
#include <QWheelEvent>

DiagramView::DiagramView(QWidget *parent) : QGraphicsView(parent)
{
    // Make it so that transformations (essentially zooming) are centered on mouse
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
}

void DiagramView::wheelEvent(QWheelEvent *wheelEvent)
{
    // Zoom if CTRL is pressed while scrolling
    if (wheelEvent->modifiers() & Qt::ControlModifier) {

        // Handle direction of zoom
        if (wheelEvent->delta() > 0)
            scale(SCALE_FACTOR, SCALE_FACTOR);
        else
            scale(1 / SCALE_FACTOR, 1 / SCALE_FACTOR);
    } else {
        // If CTRL is not pressed, simply scroll
        QGraphicsView::wheelEvent(wheelEvent);
    }
}

