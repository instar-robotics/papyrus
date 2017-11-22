#include "diagrambox.h"
#include <iostream>
#include <QPen>

DiagramBox::DiagramBox(QGraphicsItem *parent) : QGraphicsRectItem(parent)
{
    startLine = 0;
    endLine = 0;
    setRect(QRectF(0, 0, 150, 100));
    setPen((QPen(Qt::black, 2)));
    setFlag(QGraphicsItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
}

bool DiagramBox::setStartLine(Arrow *line)
{
    if (startLine)
        return false;

    startLine = line;
    return true;
}

bool DiagramBox::setEndLine(Arrow *line)
{
    if (endLine)
        return false;

    endLine = line;
    return true;
}

QVariant DiagramBox::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    if (change == QGraphicsItem::ItemPositionChange && scene()) {
        QPointF newPos = value.toPointF();
        QPointF newCenter = boundingRect().center();
        QPointF newPoint = newPos + newCenter;

        if (startLine)
            startLine->updatePosition(newPoint, true);

        if (endLine)
            endLine->updatePosition(newPoint, false);
    }

    return QGraphicsItem::itemChange(change, value);
}

/*
void DiagramBox::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    // Make selected boxes appear "bold"
    if (isSelected()) {
        QPen p = pen();
        p.setWidth(4);
        setPen(p);
    } else {
        QPen p = pen();
        p.setWidth(2);
        setPen(p);
    }

    QGraphicsRectItem::paint(painter, option, widget);
}
//*/

