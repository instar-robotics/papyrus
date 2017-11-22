#include "diagrambox.h"
#include <iostream>
#include <QPen>

DiagramBox::DiagramBox(QGraphicsItem *parent) : QGraphicsRectItem(parent)
{
    setRect(QRectF(0, 0, 150, 100));
    setPen((QPen(Qt::black, 2)));
    setFlag(QGraphicsItem::ItemIsMovable);
}

DiagramBox::DiagramBox(qreal x, qreal y, qreal width, qreal height) : QGraphicsRectItem(x, y, width, height)
{
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

