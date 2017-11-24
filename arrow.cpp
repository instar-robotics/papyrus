#include "arrow.h"
#include "constants.h"

#include <iostream>
#include <QGraphicsScene>
#include <QPainter>
#include <math.h>

int Arrow::nb = 0;

Arrow::Arrow(QGraphicsItem *parent) : QGraphicsLineItem(parent)
{
    no = nb;
    from_ = 0;
    to_ = 0;
    nb += 1;
    setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap));
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

QPainterPath Arrow::shape() const
{
    QPainterPath path = QGraphicsLineItem::shape();
    path.addPolygon(arrowHead);

    return path;
}

void Arrow::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    qreal arrowSize = 10;
    qreal angle = acos(line().dx() / line().length());

    if (line().dy() > 0)
        angle = 2 * M_PI - angle;

    QPointF arrowP1 = line().p2() - QPointF(sin(angle + M_PI / 3) * arrowSize,
                                            cos(angle + M_PI / 3) * arrowSize);

    QPointF arrowP2 = line().p2() - QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize,
                                            cos(angle + M_PI - M_PI / 3) * arrowSize);

    arrowHead.clear();
    arrowHead << line().p2() << arrowP1 << arrowP2;

    painter->setBrush(QBrush(Qt::blue));
    painter->drawLine(line());
    painter->drawPolygon(arrowHead);
}
