#include "diagrambox.h"
#include <iostream>
#include <QPen>

int DiagramBox::nb = 0;

DiagramBox::DiagramBox(QGraphicsItem *parent) : QGraphicsRectItem(parent)
{
    no = nb;
    startLine_ = 0;
    endLine_ = 0;
    setRect(QRectF(0, 0, 150, 100));
    setPen((QPen(Qt::black, 2)));
    setFlags(QGraphicsItem::ItemIsSelectable
             | QGraphicsItem::ItemIsMovable |
             QGraphicsItem::ItemSendsScenePositionChanges);
    nb += 1;
}

/*
 * Add an Arrow that originates from this Box
 */
void DiagramBox::addStartLine(Arrow *line)
{
    startLines_.insert(line);
}

/*
 * Add an Arrow that points to this Box
 */
void DiagramBox::addEndLine(Arrow *line)
{
    endLines_.insert(line);
}

/*
 * Remove the given Arrow from the list of starting lines
 */
void DiagramBox::removeStartLine(Arrow *line)
{
    startLines_.erase(startLines_.find(line));
}

/*
 * Remove the given Arrow from the list of ending lines
 */
void DiagramBox::removeEndLine(Arrow *line)
{
    endLines_.erase(endLines_.find(line));
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

