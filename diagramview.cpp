#include "diagramview.h"

#include <iostream>
#include <QWheelEvent>

DiagramView::DiagramView(QWidget *parent) : QGraphicsView(parent)
{

}

void DiagramView::wheelEvent(QWheelEvent *wheelEvent)
{
    if (wheelEvent->modifiers() & Qt::ControlModifier) {

        if (wheelEvent->delta() > 0)
            scale(1.1, 1.1);
        else
            scale(1/1.1, 1/1.1);
    } else {
        QGraphicsView::wheelEvent(wheelEvent);
    }
}

