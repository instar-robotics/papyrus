#include "arrow.h"
#include "constants.h"

#include <iostream>
#include <QGraphicsScene>
#include <QPainter>
#include <math.h>
#include <QDebug>

#include <diagrambox.h>

int Arrow::nb = 0;

int Arrow::getType()
{
    return UserType + 2;
}

Arrow::Arrow(QGraphicsItem *parent) : QGraphicsLineItem(parent),
                                      no(nb),
                                      m_from(NULL),
                                      m_to(NULL)
{
//    no = nb;
//    from_ = 0;
//    to_ = 0;
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

void Arrow::setFrom(OutputSlot *box)
{
    m_from = box;
}

void Arrow::setTo(InputSlot *box)
{
    m_to = box;
}

QPainterPath Arrow::shape() const
{
    QPainterPath path = QGraphicsLineItem::shape();
    path.addPolygon(arrowHead);

    return path;
}

int Arrow::type() const
{
    return Arrow::getType();
}

void Arrow::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    qreal arrowSize = 10;
    qreal angle = acos(line().dx() / line().length());
    InputSlot *boxTo = to();
    OutputSlot *boxFrom = from();

    if (line().dy() > 0)
        angle = 2 * M_PI - angle;

    arrowHead.clear();

    painter->setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap));

    // Check if the arrow has both a destination and an origin
    if (boxTo == NULL || boxFrom == NULL) {
       qFatal("ERROR: arrow is missing a destination and/or an origin");
    }

    if (boxTo->box() != boxFrom->box()) {
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
        // Change the angle so that it matches an almost horizontal
        angle = angle - M_PI + M_PI / 8;

        // Get the target slot's bounding rect for reference
        QRectF destRect = boxTo->boundingRect();
        QRectF originRect = boxFrom->boundingRect();

        // Get origin and destination points
        QPointF destPt = boxTo->mapRectToItem(this, destRect).center();
        QPointF origPt = boxFrom->mapRectToItem(this, originRect).center();

        // Compute control points c1 and c2 for the bezier curve by tracing lines between the origin
        // and destination points and taking the normal vectors ; then the p2 points are slightly
        // moved away on the horizontal axis

        QLineF directLine(origPt, destPt);
        QLineF normalEnd = directLine.normalVector();
        normalEnd.setAngle(normalEnd.angle() + 180); // reverse direction because it goes down

        QLineF directLine2(destPt, origPt);
        QLineF normalStart = directLine2.normalVector();

        // Move extremities away from each other on the X axis
        QPointF c1 = normalEnd.p2();
        c1.rx() += 100;
        QPointF c2 = normalStart.p2();
        c2.rx() -= 100;

        // Draw the bezier as a path
        QPainterPath curvePath;
        curvePath.moveTo(origPt);
        curvePath.cubicTo(c1, c2, destPt);
        painter->drawPath(curvePath);

        // Create the points for the arrow head
        QPointF headCenter = (c1 + c2) / 2;
        headCenter.ry() += 47;
//        QPointF arrowP1 = line().p2() + QPointF(sin(angle + M_PI / 4) * arrowSize,
        QPointF arrowP1 = headCenter + QPointF(sin(angle + M_PI / 4) * arrowSize,
                                                cos(angle + M_PI / 4) * arrowSize);

//        QPointF arrowP2 = line().p2() + QPointF(sin(angle + M_PI - M_PI / 2) * arrowSize,
        QPointF arrowP2 = headCenter + QPointF(sin(angle + M_PI - M_PI / 2) * arrowSize,
                                                cos(angle + M_PI - M_PI / 2) * arrowSize);


//        arrowHead << line().p2() << arrowP1 << arrowP2;
        arrowHead << headCenter << arrowP1 << arrowP2;
//        painter->drawLine(QLineF(line().p2(), arrowP2));
    }

    painter->drawPolygon(arrowHead);
}
