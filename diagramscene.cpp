#include "diagramscene.h"
//#include "arrow.h"
//#include "diagrambox.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <diagrambox.h>
#include <arrow.h>
#include <iostream>
#include <QKeyEvent>
#include <QList>

DiagramScene::DiagramScene(QObject *parent) : QGraphicsScene(parent)
{
    middleBtnIsDown = false;
    line = 0;
    box = 0;
}

//*
DiagramScene::~DiagramScene()
{
    delete line;
    delete box;
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
            DiagramBox *maybeItem = qgraphicsitem_cast<DiagramBox *>(itemAt(evt->scenePos(), QTransform()));

            if (maybeItem) {
                // If we have released on top on something, create an Arrow between the two items
                QPointF center = maybeItem->boundingRect().center();
                QPointF endPoint = maybeItem->scenePos() + center;

                Arrow *finalLine = new Arrow(QLineF(line->line().p1(), endPoint));
                finalLine->setPen(QPen(Qt::blue, 2, Qt::SolidLine, Qt::RoundCap));

                box->setStartLine(finalLine);
                maybeItem->setEndLine(finalLine);

                // Link the newly-created Arrow with its corresponding DiagramBoxes
                box->setStartLine(finalLine);
                maybeItem->setEndLine(finalLine);
                finalLine->setFrom(box);
                finalLine->setTo(maybeItem);

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

void DiagramScene::keyPressEvent(QKeyEvent *evt)
{
    // Delete selected items when 'DELETE' is pressed
    if (evt->key() == Qt::Key_Delete) {
        QList<QGraphicsItem *> items = selectedItems();

        int nbItems = items.count();
        for (int i = 0; i < nbItems; i += 1) {
            // Remove the item
            removeItem(qgraphicsitem_cast<DiagramBox *>(items.at(i)));
        }
    }

    QGraphicsScene::keyPressEvent(evt);
}

void DiagramScene::removeItem(QGraphicsItem *item)
{
    QGraphicsScene::removeItem(item);
}

void DiagramScene::removeItem(Arrow *arrow)
{
    QGraphicsScene::removeItem(arrow);
}

/*
 * Remove a DiagramBox from the scene and also remove any lines that were connected to it
 */

void DiagramScene::removeItem(DiagramBox *box)
{
    /*
     * ATTENTION: check for leaks:
     * - should we delete the pointers?
     * - Any risks of double deletion?
     * - Should we restore the pointer to 0?
     */
    /*
    if (box->startLine() && box->startLine()->scene()) {
        QGraphicsScene::removeItem(box->startLine());
        box->setStartLine(0);
    }

    if (box->endLine() && box->endLine()->scene()) {
        QGraphicsScene::removeItem(box->endLine());
        box->setEndLine(0);
    }
    //*/

    if (box->startLine()) {
        Arrow *line = box->startLine();
        DiagramBox *endBox = line->to();
        // Unlink the Arrow from its DiagramBoxes
        line->setTo(NULL);
        line->setFrom(NULL);
        endBox->setEndLine(NULL); // Before removing the Arrow, we remove it from its end box
        box->setStartLine(NULL);  // Remove the Arrow from this box

        removeItem(line);      // Remove the Arrow from the scene
        delete line;           // Delete the Arrow
    }

    if (box->endLine()) {
        Arrow *line = box->endLine();
        DiagramBox *startBox = line->from();
        // Unlink the Arrow from its DiagramBoxes
        line->setTo(NULL);
        line->setFrom(NULL);
        startBox->setStartLine(NULL); // Before removing the Arrow, we remove it from its end box
        box->setEndLine(NULL);        // Remove the Arrow from this box

        removeItem(line);      // Remove the Arrow from the scene
        delete line;           // Delete the Arrow
    }

    QGraphicsScene::removeItem(box); // Remove the Box from the scene
    delete box;                      // Delete the Box
}
