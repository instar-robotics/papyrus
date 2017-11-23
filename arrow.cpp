#include "arrow.h"

#include <iostream>
#include <QGraphicsScene>

int Arrow::nb = 0;

Arrow::Arrow(QGraphicsItem *parent) : QGraphicsLineItem(parent)
{
    no = nb;
    from_ = 0;
    to_ = 0;
    nb += 1;
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

void Arrow::setFrom(DiagramBox *box)
{
    from_ = box;
}

void Arrow::setTo(DiagramBox *box)
{
    to_ = box;
}
