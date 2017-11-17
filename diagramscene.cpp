#include "diagramscene.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <diagrambox.h>
#include <iostream>

DiagramScene::DiagramScene(const QString &name, QObject *parent) : QGraphicsScene(parent)
{
    sceneName = new QString(name);
    setSceneRect(0, 0, 5000, 5000);
    leftBtnIsDown = false;
    middleBtnIsDown = false;
    line = 0;
}

DiagramScene::~DiagramScene()
{
    delete sceneName;
}

void DiagramScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() == Qt::MiddleButton) {

        middleBtnIsDown = true;

        // Check if we have clicked on something
        QGraphicsItem *maybeItem = itemAt(mouseEvent->scenePos(), QTransform());
        if (!maybeItem) {
            // Only insert mode for now
            //QGraphicsRectItem *item = new QGraphicsRectItem(0, 0, 150, 100);
            DiagramBox *item = new DiagramBox;
            //item->setFlag(QGraphicsItem::ItemIsSelectable);

            addItem(item);
            QPointF p = item->boundingRect().bottomRight();
            item->setPos(mouseEvent->scenePos() - p / 2);
        } else {
            QPointF start = maybeItem->scenePos();
            QRectF box = maybeItem->boundingRect();
            qreal w = box.width();
            qreal h = box.height();
            QPointF end = QPointF(start.x() + w, start.y() + h);

            line = new QGraphicsLineItem(QLineF((start + end) / 2,
                                         mouseEvent->scenePos()));
            addItem(line);
        }
    }

    QGraphicsScene::mousePressEvent(mouseEvent);
}

void DiagramScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (middleBtnIsDown && line != 0) {
        QLineF newLine(line->line().p1(), mouseEvent->scenePos());
        line->setLine(newLine);
    } else {
        // No nothing for now
        QGraphicsScene::mouseMoveEvent(mouseEvent);
    }
}

void DiagramScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() == Qt::MiddleButton) {

        middleBtnIsDown = false;

        if (line != 0) {
            removeItem(line);

            // Check if we have released on something
            QGraphicsItem *maybeItem = itemAt(mouseEvent->scenePos(), QTransform());

            if (maybeItem) {
                QPointF start = maybeItem->scenePos();
                QRectF box = maybeItem->boundingRect();
                qreal w = box.width();
                qreal h = box.height();
                QPointF end = QPointF(start.x() + w, start.y() + h);
                QGraphicsLineItem *finalLine = new QGraphicsLineItem(QLineF(line->line().p1(), (start + end) / 2));
                finalLine->setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap));
                addItem(finalLine);
            }

            delete line;
            line = 0;
        }
    } else if (mouseEvent->button() == Qt::RightButton) {
        std::cout << "Nb items selected: " << selectedItems().count() << std::endl;
    /*} else if (mouseEvent->button() == Qt::LeftButton) {
        QGraphicsItem *maybeItem = itemAt(mouseEvent->scenePos(), QTransform());

        if (maybeItem) {
            maybeItem->setSelected(!maybeItem->isSelected());
        }//*/
    }
}
