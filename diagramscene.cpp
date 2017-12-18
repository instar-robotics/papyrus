#include "diagramscene.h"
#include "librarypanel.h"
#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "outputslot.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <diagrambox.h>
#include <arrow.h>
#include <iostream>
#include <QKeyEvent>
#include <QList>
#include <QMimeData>
#include <QGraphicsView>
#include <QApplication>
#include <QDebug>

DiagramScene::DiagramScene(QObject *parent) : QGraphicsScene(parent),
                                            middleBtnIsDown(false),
                                            m_shouldDrawGrid(true),
                                            m_gridSize(35),
                                            line(NULL),
                                            box(NULL),
                                            m_script(NULL)
{

}

DiagramScene::~DiagramScene()
{
    delete line;
    delete box;
}

DiagramBox *DiagramScene::addBox(const QPointF &position, const QString &name, const QIcon &icon, OutputSlot *outputSlot, QUuid uuid)
{
    Q_ASSERT(outputSlot != NULL);

    // Create the box itself, the body of the block
    DiagramBox *newBox = new DiagramBox(name, icon, outputSlot, uuid);
    QPointF center = newBox->boundingRect().center();
    newBox->setPos(position - center);

    addItem(newBox);

    /*
    // Set the output slot's position
    QPointF osPos = newBox->pos();
    osPos.rx() += newBox->boundingRect().right();
    osPos.ry() += newBox->boundingRect().bottom() / 2;
    osPos.ry() -= outputSlot->boundingRect().bottom() / 2;
    outputSlot->setPos(osPos);

    // Create the output slot
    addItem(newBox);
    addItem(outputSlot);
    //*/

    // Set the new script as modified
    m_script->setStatusModified(true);

    return newBox;
}

/*
 * Check the items's bounding rects and update the scene's 'sceneRect' to contain them all.
 * Default to a minimum size of the container widget's size (this is to prevent weird behavior
 * when trying to add items in a small scene: the items won't be placed under the mouse)
 */
void DiagramScene::updateSceneRect()
{
    // First, get the main window
    PapyrusWindow *mainWindow = NULL;

    foreach (QWidget *w, qApp->topLevelWidgets()) {
        if (PapyrusWindow *mW = qobject_cast<PapyrusWindow *>(w)) {
            mainWindow = mW;
            break;
        }
    }

    if (mainWindow) {
        // Now get the tab widget's size: this will be the minimum size we want
        QSize widgetSize = mainWindow->getUi()->tabWidget->size();

        // Get the bounding rect of all items
        QRectF bounds = itemsBoundingRect();

        // Get the current sceneRect
        QRectF sRect = sceneRect();

        if (bounds.isEmpty() || bounds.isNull()) {
             // If there is no elements, resize the scene to the widget size and center on 0
            QRectF initial(- widgetSize.width() / 2,
                           - widgetSize.height() / 2,
                           widgetSize.width(),
                           widgetSize.height());

            setSceneRect(initial);
        } else if (!sRect.contains(bounds)) {
            // If there are elements, resize the scene if some are outside it
            QRectF newSize;
            newSize.setX(bounds.x() - SCENE_RECT_MARGIN);
            newSize.setY(bounds.y() - SCENE_RECT_MARGIN);
            newSize.setWidth(bounds.width() + 2 * SCENE_RECT_MARGIN);
            newSize.setHeight(bounds.height() + 2 * SCENE_RECT_MARGIN);

            setSceneRect(newSize);
        }
    }
}

void DiagramScene::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    if (evt->button() & Qt::MiddleButton) {
        middleBtnIsDown = true;

        // Check if we have clicked on something
        QGraphicsItem *maybeItem = itemAt(evt->scenePos(), QTransform());
        if (!maybeItem) {
            // If we clicked on empty space, add an item centered on the mouse cursor

//            addBox(evt->scenePos());

            updateSceneRect();
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
            updateSceneRect();
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

                // TODO: create a function to "add an arrow" instead of doing this
                Arrow *finalLine = new Arrow(QLineF(line->line().p1(), endPoint));
                // Link the newly-created Arrow with its corresponding DiagramBoxes
                box->addStartLine(finalLine);
                maybeItem->addEndLine(finalLine);
                finalLine->setFrom(box);
                finalLine->setTo(maybeItem);

                addItem(finalLine);

                // Set associated script as modified
                m_script->setStatusModified(true);
            }

            // Delete the current line (whether or not we created the final line)
            delete line;
            line = 0;
            box = 0;
        }
    }

    updateSceneRect();

    // Important! Otherwise the box's position doesn't get updated.
    QGraphicsScene::mouseReleaseEvent(evt);
}

void DiagramScene::dragEnterEvent(QGraphicsSceneDragDropEvent *evt)
{
    if (evt->mimeData()->hasFormat(LibraryPanel::libraryItemMimeType())) {
        QByteArray pieceData = evt->mimeData()->data(LibraryPanel::libraryItemMimeType());
        QDataStream dataStream(&pieceData, QIODevice::ReadOnly);
        QString name;
        QIcon icon;

        dataStream >> name >> icon;
        QString str(tr("Add '%1' in script...").arg(name));
        emit(displayStatusMessage(str));
        setBackgroundBrush(QBrush(QColor(220, 220, 220, 50)));
    } else {
        // EMIT SIGNAL TO STATUS MESSAGE BAR
        evt->ignore();
    }
}

void DiagramScene::dragLeaveEvent(QGraphicsSceneDragDropEvent *evt)
{
    if (evt->mimeData()->hasFormat(LibraryPanel::libraryItemMimeType())) {
        QByteArray pieceData = evt->mimeData()->data(LibraryPanel::libraryItemMimeType());
        QDataStream dataStream(&pieceData, QIODevice::ReadOnly);
        QString name;
        QIcon icon;

        dataStream >> name >> icon;
        QString str(tr("Cancel adding '%1' in script...").arg(name));

        emit(displayStatusMessage(str));
        setBackgroundBrush(QBrush(Qt::white));
    } else {
        evt->ignore();
        emit(displayStatusMessage("Unsupported drop event, discarding."));
    }
}

void DiagramScene::dragMoveEvent(QGraphicsSceneDragDropEvent *evt)
{
    if (evt->mimeData()->hasFormat(LibraryPanel::libraryItemMimeType())) {
    } else {
        evt->ignore();
        emit(displayStatusMessage("Unsupported drop event, discarding."));
    }
}

void DiagramScene::dropEvent(QGraphicsSceneDragDropEvent *evt)
{
    if (evt->mimeData()->hasFormat(LibraryPanel::libraryItemMimeType())) {
        // If this is a drop from the library
        QByteArray pieceData = evt->mimeData()->data(LibraryPanel::libraryItemMimeType());
        QDataStream dataStream(&pieceData, QIODevice::ReadOnly);
        QString name;
        QString descriptionFile;
        QString outputName;
        QIcon icon;
//        std::vector<InputSlot> inputs;
//        int nb;

        dataStream >> name >> icon >> descriptionFile >> outputName;

        OutputSlot *outputSlot = new OutputSlot(outputName);
        DiagramBox *b = addBox(evt->scenePos(), name, icon, outputSlot);
        b->setDescriptionFile(descriptionFile);

        setBackgroundBrush(QBrush(Qt::white));
        QString str(tr("Function '%1' added in script").arg(name));
        emit(displayStatusMessage(str));
    } else {
        evt->ignore();
        emit(displayStatusMessage("Unsupported drop event, discarding."));
    }
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
            updateSceneRect();
        }

        // Set the associated script as modified if there was a deletion
        if (nbItems > 0)
            m_script->setStatusModified(true);
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

    std::cout << "About to delete box #??" << std::endl;

    // TODO: re-implement this in a more efficient manner!
    std::cout << "Box #?? has " << box->startLines().size() << " start lines" << std::endl;

    for (auto line : box->startLines()) {
        std::cout << "Dealing with line #" << line->no << std::endl;
        DiagramBox *endBox = line->to();
        std::cout << "endBox for this line is #??" << std::endl;

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
   std::cout << "Box #??? has " << box->endLines().size() << " end lines" << std::endl;

   for (auto line : box->endLines()) {
       std::cout << "Dealing with line #" << line->no << std::endl;
       DiagramBox *startBox = line->from();
       std::cout << "startBox for this line is #??" << std::endl;

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

/**
 * @brief Draw the grid if the option is set
 * @param painter the painter object, used to pain
 * @param rect the portion of the scene that is currently visible
 */
void DiagramScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    if (!m_shouldDrawGrid)
        return;

    QPen pen;
    painter->setPen(pen);

    qreal left = int(rect.left()) - (int(rect.left()) % m_gridSize);
    qreal top = int(rect.top()) - (int(rect.top()) % m_gridSize);

    QVector<QPointF> dots;

    for (qreal x = left; x < rect.right(); x += m_gridSize) {
        for (qreal y = top; y < rect.bottom(); y += m_gridSize) {
            dots.append(QPointF(x, y));
        }
    }

    painter->drawPoints(dots.data(), dots.size());
}

Script *DiagramScene::script() const
{
    return m_script;
}

void DiagramScene::setScript(Script *script)
{
    m_script = script;
}

int DiagramScene::gridSize() const
{
    return m_gridSize;
}

bool DiagramScene::shouldDrawGrid() const
{
    return m_shouldDrawGrid;
}

void DiagramScene::setShouldDrawGrid(bool shouldDrawGrid)
{
    m_shouldDrawGrid = shouldDrawGrid;
}

void DiagramScene::toggleDisplayGrid(bool shouldDraw)
{
    m_shouldDrawGrid = shouldDraw;
    update(sceneRect());
}
