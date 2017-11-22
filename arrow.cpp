#include "arrow.h"

#include <iostream>
#include <QGraphicsScene>

Arrow::Arrow(QGraphicsItem *parent) : QGraphicsLineItem(parent)
{

}

Arrow::Arrow(const QLineF &line, QGraphicsItem *parent) : Arrow(parent)
{
    setLine(line);
}

void Arrow::updatePosition(QPointF newPoint, bool isStartPoint)
{
    QPointF p1 = isStartPoint ? newPoint : line().p1();
    QPointF p2 = isStartPoint ? line().p2() : newPoint;

    setLine(QLineF(p1, p2));
}

void Arrow::boxDeleted() {
    scene()->removeItem(this);
    emit(deleted());
}
