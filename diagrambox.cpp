#include "diagrambox.h"
#include <iostream>
#include <QPen>

int DiagramBox::nb = 0;

DiagramBox::DiagramBox(QGraphicsItem *parent) : QGraphicsRectItem(parent),
                                                startLines_(),
                                                endLines_()
{
    no = nb;
    setRect(QRectF(0, 0, 150, 100));
    setPen((QPen(Qt::black, 2)));
    setFlags(QGraphicsItem::ItemIsSelectable
           | QGraphicsItem::ItemIsMovable
           | QGraphicsItem::ItemSendsScenePositionChanges);
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

/*
 * React to when the DiagramBox experiences a change
 */
QVariant DiagramBox::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    // When it is moved, we need to move its connected Arrows
    if (change == QGraphicsItem::ItemPositionChange && scene()) {
        QPointF newPos = value.toPointF();

        QPointF newStartPoint = newPos;
        newStartPoint.rx() += boundingRect().width();
        newStartPoint.ry() += boundingRect().height() / 2;

        QPointF newEndPoint = newPos;
        newEndPoint.ry() += boundingRect().height() / 2;
//        QPointF newCenter = boundingRect().center();
//        QPointF newPoint = newPos + newCenter;

        // Update position of start lines
        for (auto line : startLines_) {
            line->updatePosition(newStartPoint, true);
        }

        // Update position of end lines
        for (auto line : endLines_) {
            line->updatePosition(newEndPoint, false);
        }
    } else if (change == QGraphicsItem::ItemSelectedHasChanged && isSelected()) {
        std::cout << std::endl;
        std::cout << "Box #" << no << " selected, it has " << startLines().size()
                  << " start lines and " << endLines().size() << " end lines." << std::endl;
    }

    return QGraphicsItem::itemChange(change, value);
}

//*
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

