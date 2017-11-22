#include "diagramscene.h"
//#include "arrow.h"
//#include "diagrambox.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <diagrambox.h>
#include <arrow.h>
#include <iostream>

DiagramScene::DiagramScene(QObject *parent) : QGraphicsScene(parent)
{
    middleBtnIsDown = false;
    line = 0;
    box = 0;
    /*
    sceneName = new QString(name);
    setSceneRect(0, 0, 5000, 5000);
    leftBtnIsDown = false;
    middleBtnIsDown = false;
    line = 0;
    //*/
}

/*
DiagramScene::~DiagramScene()
{
    delete sceneName;
}
//*/

void DiagramScene::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    if (evt->button() & Qt::MiddleButton) {
        middleBtnIsDown = true;

        // Check if we have clicked on something
        QGraphicsItem *maybeItem = itemAt(evt->scenePos(), QTransform());
        if (!maybeItem) {
            // If we clicked on empty space, add an item centered on the mouse cursor
            DiagramBox *newBox = new DiagramBox;
            QPointF center = newBox->boundingRect().center();
            newBox->setPos(evt->scenePos() - center);
            addItem(newBox);
        } else {
            // If we clicked on an item, start drawing a line from the center of the item
            QPointF center = maybeItem->boundingRect().center();
            line = new QGraphicsLineItem(QLineF(maybeItem->scenePos() + center,
                                         evt->scenePos()));
            line->setPen(QPen(Qt::black, 1, Qt::DashLine));
            box = qgraphicsitem_cast<DiagramBox *>(maybeItem);
            addItem(line);
        }
    }

    QGraphicsScene::mousePressEvent(evt);
}

void DiagramScene::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
    if (middleBtnIsDown && line != 0 && box != 0) {
        QLineF newLine(line->line().p1(), evt->scenePos());
        line->setLine(newLine);
    }

    QGraphicsScene::mouseMoveEvent(evt);

}

void DiagramScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt) {
    if (evt->button() == Qt::MiddleButton) {
        middleBtnIsDown = false;

        if (line != 0) {
            removeItem(line);

            // Check if we have released on top of an item
            //QGraphicsItem *maybeItem = itemAt(evt->scenePos(), QTransform());
            DiagramBox *maybeItem = qgraphicsitem_cast<DiagramBox *>(itemAt(evt->scenePos(), QTransform()));

            if (maybeItem) {
                // If we have released on top on something, create an Arrow between the two items
                QPointF center = maybeItem->boundingRect().center();
                QPointF endPoint = maybeItem->scenePos() + center;

                Arrow *finalLine = new Arrow(QLineF(line->line().p1(), endPoint));
                finalLine->setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap));

                box->setStartLine(finalLine);
                maybeItem->setEndLine(finalLine);

                //connect(maybeItem, SIGNAL(positionChanged(false)), finalLine, SLOT(updatePosition(bool)));
                //connect(box, SIGNAL(positionChanged(true)), finalLine, SLOT(updatePosition(bool)));

                addItem(finalLine);
            }

            // Delete the current line (whether or not we created the final line)
            delete line;
            line = 0;
            box = 0;
        }
    }

    // Important! Otherwise the box's position doesn't get updated.
    QGraphicsScene::mouseReleaseEvent(evt);
}
