#include "arrow.h"
#include "constants.h"

#include <iostream>
#include <QGraphicsScene>
#include <QPainter>
#include <math.h>

#include <diagrambox.h>

int Arrow::nb = 0;

Arrow::Arrow(QGraphicsItem *parent) : QGraphicsLineItem(parent)
{
    no = nb;
    from_ = 0;
    to_ = 0;
    nb += 1;
//    setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap));
//    setFlags(QGraphicsItem::ItemSendsScenePositionChanges | QGraphicsItem::ItemIsSelectable);
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

    arrowHead.clear();

    painter->setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap));

    if (to() != from()) {
        // Paint a normal line if the start and end boxes are different
        painter->drawLine(line());

        QPointF middle = (line().p1() + line().p2()) / 2;

        QPointF arrowP1 = middle - QPointF(sin(angle + M_PI / 3) * arrowSize,
                                                cos(angle + M_PI / 3) * arrowSize);

        QPointF arrowP2 = middle - QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize,
                                                cos(angle + M_PI - M_PI / 3) * arrowSize);

        arrowHead << middle << arrowP1 << arrowP2;
    } else {
        // Paint an arc line if the box is connected to itself
        // Get the bounding rect of the associated box for reference
        QRectF arcRect = to()->boundingRect();
        QRectF lineRect = to()->mapRectToItem(this, arcRect); // Map the coordinates to the item's
        // Make adjustements so that the arc is drawn above the box
        lineRect.adjust(-lineRect.width() / 4,
                        -lineRect.height() / 2,
                        lineRect.width() / 4,
                        -lineRect.height() / 3);

        // Angle values adjusted for pixel precision
        int startAngle = -48 * 16;
        int spanAngle = 276 * 16;

        // Draw the arc
        painter->drawArc(lineRect, startAngle, spanAngle);

        QPointF arrowP1 = line().p2() + QPointF(sin(angle + M_PI / 4) * arrowSize,
                                                cos(angle + M_PI / 4) * arrowSize);

        QPointF arrowP2 = line().p2() + QPointF(sin(angle + M_PI - M_PI / 2) * arrowSize,
                                                cos(angle + M_PI - M_PI / 2) * arrowSize);


        arrowHead << line().p2() << arrowP1 << arrowP2;
//        painter->drawLine(QLineF(line().p2(), arrowP2));
    }

    painter->drawPolygon(arrowHead);
}
