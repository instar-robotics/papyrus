#include "diagrambox.h"
#include <iostream>
#include <QPen>

DiagramBox::DiagramBox(QGraphicsItem *parent) : QGraphicsRectItem(parent)
{
    startLine_ = 0;
    endLine_ = 0;
    setRect(QRectF(0, 0, 150, 100));
    setPen((QPen(Qt::black, 2)));
    setFlags(QGraphicsItem::ItemIsSelectable
             | QGraphicsItem::ItemIsMovable |
             QGraphicsItem::ItemSendsScenePositionChanges);
}

bool DiagramBox::setStartLine(Arrow *line)
{
    if (startLine_)
        return false;

    startLine_ = line;
    return true;
}

bool DiagramBox::setEndLine(Arrow *line)
{
    if (endLine_)
        return false;

    endLine_ = line;
    return true;
}

QVariant DiagramBox::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    if (change == QGraphicsItem::ItemPositionChange && scene()) {
        QPointF newPos = value.toPointF();
        QPointF newCenter = boundingRect().center();
        QPointF newPoint = newPos + newCenter;

        if (startLine_)
            startLine_->updatePosition(newPoint, true);

        if (endLine_)
            endLine_->updatePosition(newPoint, false);
    } else if (change == QGraphicsItem::ItemSceneHasChanged && !scene()) {
        // When the box is removed from a scene
        emit(deleted());
    }

    return QGraphicsItem::itemChange(change, value);
}

void DiagramBox::startLineDeleted()
{
    // ATTENTION: to prevent leak, we must make sure to delete the pointer to the Arrow
    startLine_ = 0;
}

void DiagramBox::endLineDeleted()
{
    // ATTENTION: to prevent leak, we must make sure to delete the pointer to the Arrow
    endLine_ = 0;
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

