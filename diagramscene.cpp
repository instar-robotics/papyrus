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
            QPointF startPoint = maybeItem->scenePos();
            startPoint.ry() += maybeItem->boundingRect().bottom() / 2;
            startPoint.rx() += maybeItem->boundingRect().right();

            line = new QGraphicsLineItem(QLineF(startPoint,
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
                QPointF endPoint = maybeItem->scenePos();
                endPoint.ry() += maybeItem->boundingRect().bottom() / 2;


                Arrow *finalLine = new Arrow(QLineF(line->line().p1(), endPoint));
                // Link the newly-created Arrow with its corresponding DiagramBoxes
                box->addStartLine(finalLine);
                maybeItem->addEndLine(finalLine);
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
    // ATTENTION: check for leaks: do we need to delete the items?

    std::cout << "About to delete box #" << box->no << std::endl;

    // TODO: re-implement this in a more efficient manner!
    std::cout << "Box #" << box->no << " has " << box->startLines().size() << " start lines" << std::endl;

    for (auto line : box->startLines()) {
        std::cout << "Dealing with line #" << line->no << std::endl;
        DiagramBox *endBox = line->to();
        std::cout << "endBox for this line is #" << endBox->no << std::endl;

        // Unlink the Arrow from its DiagramBoxes
        line->setTo(NULL);
        line->setFrom(NULL);

        endBox->removeEndLine(line); // Remove this Arrow from the endBox's endlines
        box->removeStartLine(line);  // Remove this Arrow from this box's start lines

        removeItem(line); // Remove the line from the scene
    }

    // Empty the list of start lines
    // ATTENTION: do we need to explicitly call 'delete' or will 'erase()' do it for us?
    box->startLines().clear();

    // TODO: re-implement this in a more efficient manner!
   std::cout << "Box #" << box->no << " has " << box->endLines().size() << " end lines" << std::endl;

   for (auto line : box->endLines()) {
       std::cout << "Dealing with line #" << line->no << std::endl;
       DiagramBox *startBox = line->from();
       std::cout << "startBox for this line is #" << startBox->no << std::endl;

       // Unlink the Arrow from its DiagramBoxes
       line->setTo(NULL);
       line->setFrom(NULL);

       startBox->removeStartLine(line); // Remove this Arrow from the endBox's endlines
       box->removeEndLine(line);  // Remove this Arrow from this box's start lines

       removeItem(line); // Remove the line from the scene
   }

   // Empty the list of start lines
   // ATTENTION: do we need to explicitly call 'delete' or will 'erase()' do it for us?
   box->endLines().clear();

    QGraphicsScene::removeItem(box); // Remove the Box from the scene
    delete box;                      // Delete the Box
}
