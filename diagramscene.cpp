#include "diagramscene.h"
#include "librarypanel.h"
#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "slot.h"
#include "helpers.h"
#include "link.h"

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
                                            m_mainWindow(NULL),
                                            m_leftBtnDown(false),
                                            middleBtnIsDown(false),
                                            m_shouldDrawGrid(true),
                                            m_gridSize(35),
                                            m_line(NULL),
                                            m_oSlot(NULL),
                                            m_script(NULL)
{
    m_mainWindow = getMainWindow();

    PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();

    connect(propPanel->okBtn(), SIGNAL(clicked(bool)), this, SLOT(onOkBtnClicked(bool)));
    connect(propPanel->cancelBtn(), SIGNAL(clicked(bool)), this, SLOT(onCancelBtnClicked(bool)));
    connect(this, SIGNAL(selectionChanged()), this, SLOT(onSelectionChanged()));
}

DiagramScene::~DiagramScene()
{
    delete m_mainWindow;
    delete m_line;
    delete m_oSlot;
}

// TODO: shouldn't this become addBox(DiagramBox *) rather?
DiagramBox *DiagramScene::addBox(const QPointF &position,
                                 const QString &name,
                                 const QIcon &icon,
                                 OutputSlot *outputSlot,
                                 std::set<InputSlot *> inputSlots,
                                 QUuid uuid)
{
    Q_ASSERT(outputSlot != NULL);

    // Create the box itself, the body of the block
    DiagramBox *newBox = new DiagramBox(name, icon, outputSlot, inputSlots, uuid);
    QPointF center = newBox->boundingRect().center();
    newBox->setPos(position - center);

    addItem(newBox);

    // Set the new script as modified
    m_script->setStatusModified(true);

    return newBox;
}

/*
 * Check the items' bounding rects and update the scene's 'sceneRect' to contain them all.
 * Default to a minimum size of the container widget's size (this is to prevent weird behavior
 * when trying to add items in a small scene: the items won't be placed under the mouse)
 */
void DiagramScene::updateSceneRect()
{
    if (m_mainWindow) {
        // Now get the tab widget's size: this will be the minimum size we want
        QSize widgetSize = m_mainWindow->getUi()->tabWidget->size();

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

/**
 * @brief DiagramScene::mousePressEvent initiates the creation of a link when the click is made
 * on an output slot.
 */
void DiagramScene::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    if (evt->button() & Qt::LeftButton) {
        m_leftBtnDown = true;

        // Check if we have clicked on something
        QGraphicsItem *maybeItem = itemAt(evt->scenePos(), QTransform());
        if (!maybeItem) {
            updateSceneRect();
        } else {
            // Check if we clicked on an output slot
            m_oSlot = dynamic_cast<OutputSlot *>(maybeItem);

            if (m_oSlot != NULL) {
                QPointF startPoint = m_oSlot->scenePos();
                m_line = new QGraphicsLineItem(QLineF(startPoint, evt->scenePos()));
                m_line->setPen(QPen(Qt::black, 1, Qt::DashLine));
                addItem(m_line);
                updateSceneRect();
            }
        }
    }
    /*
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
    //*/

    QGraphicsScene::mousePressEvent(evt);
}

/**
 * @brief DiagramScene::mouseMoveEvent update the "dist" member of the slots that are close to the
 * mouse, so that they can grow accordingly when trying to create a new link, also update the link's
 * line position (according to mouse movement) when creating a link.
 * @param evt
 */
void DiagramScene::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
    QPointF mousePos = evt->scenePos();

    // Select all Slots that are visible on the scene (in the view's viewport)
    QList<QGraphicsView *> vs = views();
    // At the moment, for simplicity, we suppose there is only one view (that will change when
    // we introduce the minimap
    if (vs.count() != 1)
        qFatal("Only one view is supported at this moment (DiagramScene::mouseMoveEvent");

    QWidget *viewport = vs[0]->viewport();
    QRect viewportRect(0, 0, viewport->width(), viewport->height());
    QRectF visibleArea = vs[0]->mapToScene(viewportRect).boundingRect();

    foreach(QGraphicsItem *item, items(visibleArea)) {
        Slot *slot = dynamic_cast<Slot *>(item);
        if (slot != NULL) {
            QPointF center = slot->scenePos();
            qreal dist = (mousePos - center).manhattanLength();
            slot->setDist(dist <= 400 ? dist : 400);
            slot->update();

            // Flag input slots that can be linked to the current output slot (if any)
            InputSlot *inputSlot = dynamic_cast<InputSlot *>(slot);
            if (inputSlot != NULL && m_leftBtnDown && m_line != NULL && m_oSlot != NULL) {
                inputSlot->setCanLink(canLink(m_oSlot->outputType(), inputSlot->inputType()));
            }
        }
    }

    // Draw a dotted line if we are creating a Link
    if (m_leftBtnDown && m_line != 0 && m_oSlot != NULL) {
        QLineF newLine(m_line->line().p1(), mousePos);
        QPen currPen = m_line->pen();

        // Update line's color and thickness based on the validity of the Link
        InputSlot *maybeSlot = dynamic_cast<InputSlot *>(itemAt(mousePos, QTransform()));
        if (maybeSlot) {
            if (canLink(m_oSlot->outputType(), maybeSlot->inputType())) {
                currPen.setColor(Qt::green);
                currPen.setWidth(2);
            } else {
                currPen.setColor(Qt::red);
                currPen.setWidth(1);
            }
        } else {
            currPen.setWidth(1);
            currPen.setColor(Qt::black);
        }

        m_line->setPen(currPen);
        m_line->setLine(newLine);
    }

    QGraphicsScene::mouseMoveEvent(evt);
}

/**
 * @brief DiagramScene::mouseReleaseEvent finalizes the creation of a link between some output
 * slots and input slots when the mouse was clicked on top of an ouput slot and released on top of
 * an input slot.
 * @param evt
 */
void DiagramScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt) {
    if (evt->button() == Qt::LeftButton) {
        m_leftBtnDown = false;

        // Remove temporary line if we were drawing one (click initiated on an output slot)
        if (m_line != 0) {
            removeItem(m_line);

            // Check if we have released on top of an input slot and create an Arrow if so
            InputSlot *maybeSlot = dynamic_cast<InputSlot *>(itemAt(evt->scenePos(), QTransform()));

            if (maybeSlot) {
                // If we have released on top on something, check that the types are compatible
                if (canLink(m_oSlot->outputType(), maybeSlot->inputType())) {
//                    QPointF endPoint = maybeSlot->scenePos();

                    /*
                    Link *zelda = new Link();
                    zelda->setFrom(m_oSlot);
                    zelda->setTo(maybeSlot);
                    m_oSlot->addOutput(zelda);
                    addItem(zelda);

                    qDebug() << "Link added";
                    //*/

                    /*
                    Arrow *newLine = new Arrow(QLineF(m_line->line().p1(), endPoint));
                    m_oSlot->addOutput(newLine);
                    maybeSlot->addInput(newLine);
                    newLine->setFrom(m_oSlot);
                    newLine->setTo(maybeSlot);
                    addItem(newLine);
                    //*/
                    // TODO: set script status modified
                } else {
                    emit displayStatusMessage("Invalid connection!");
                }
            }

            delete m_line;
            m_line = 0;
        }

        m_oSlot = 0;
    }

    if (evt->button() == Qt::MiddleButton) {
        /*
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
        //*/
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
        OutputType outputType;
        qint32 outputType_;
        QIcon icon;
        int nbInputs;

        // Then proceed to retrieve the other elements
        dataStream >> name >> icon >> descriptionFile >> outputName >> outputType_ >> nbInputs;

        // Cast the integer to the Enum type (problem of operator '>>' with enums)
        outputType = static_cast<OutputType>(outputType_);

//        std::vector<QString> inputNames;
        std::set<InputSlot *> inputSlots;
        for (int i = 0; i < nbInputs; i += 1) {
            QString iName;
            InputType inputType;
            qint32 inputType_;
            bool multiple;

            dataStream >> iName >> inputType_ >> multiple;

            // Cast the integer input type into enum InputType
            inputType = static_cast<InputType>(inputType_);

            InputSlot *iSlot = new InputSlot(iName);
            iSlot->setInputType(inputType);
            iSlot->setMultiple(multiple);
            inputSlots.insert(iSlot);

//            inputNames.push_back(iName);
        }

        OutputSlot *outputSlot = new OutputSlot(outputName);
        outputSlot->setOutputType(outputType);

        /*
        // TODO: join this with the previous foreach?
        std::set<InputSlot *> inputSlots;
        foreach (QString iName, inputNames) {
            InputSlot *iSlot = new InputSlot(iName);
            inputSlots.insert(iSlot);
        }
        //*/

        DiagramBox *b = addBox(evt->scenePos(), name, icon, outputSlot, inputSlots);
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

    qDebug() << "Needs to re-implement item deletion with associated arrows, etc.";

    /*
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
    //*/

    // Empty the list of start lines
    // ATTENTION: do we need to explicitly call 'delete' or will 'erase()' do it for us?
    box->startLines().clear();

    // TODO: re-implement this in a more efficient manner!
   std::cout << "Box #??? has " << box->endLines().size() << " end lines" << std::endl;

   /*
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
   //*/

   // Empty the list of start lines
   // ATTENTION: do we need to explicitly call 'delete' or will 'erase()' do it for us?
   box->endLines().clear();

    QGraphicsScene::removeItem(box); // Remove the Box from the scene
    delete box;                      // Delete the Box
}

/**
 * @brief Draw the grid if the option is set
 * @param painter the painter object, used to paint
 * @param rect the portion of the scene that is currently visible
 */
void DiagramScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    // Well... don't draw the grid if the option is not set
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

PapyrusWindow *DiagramScene::mainWindow() const
{
    return m_mainWindow;
}

/**
 * @brief DiagramScene::onSelectionChanged handle selection changed from the scene, for instance
 * how the fact to display a selected box or link's properties
 */
void DiagramScene::onSelectionChanged()
{
    QList<QGraphicsItem *> sItems = selectedItems();

    PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();

    if (sItems.count() == 0) {
        // Clears the PropertiesPanel is no items are selected
        propPanel->boxFrame()->hide();
        propPanel->linkFrame()->hide();
        propPanel->okBtn()->hide();
        propPanel->cancelBtn()->hide();
    } else if (sItems.count() == 1) {
        // Display a box's or link's properties only if there is only one selected
        DiagramBox *selectedBox  = qgraphicsitem_cast<DiagramBox *>(sItems.at(0));
        if (selectedBox != NULL) {
            propPanel->displayBoxProperties(selectedBox);
        } else {
            qDebug() << "Non-box items selection are not yet supported";
        }
    }
}

QGraphicsLineItem *DiagramScene::line() const
{
    return m_line;
}

bool DiagramScene::leftBtnDown() const
{
    return m_leftBtnDown;
}

Script *DiagramScene::script() const
{
    return m_script;
}

void DiagramScene::setScript(Script *script)
{
    m_script = script;
}

/**
 * @brief Create a connection between a box' output slot and another box's input slot
 * @param from: the starting output slot
 * @param to: the target input slot
 */
void DiagramScene::createLink(OutputSlot *from, InputSlot *to)
{
    qDebug() << "Should create link between" << from->name() << "and" << to->name();
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

void DiagramScene::onOkBtnClicked(bool)
{
    QList<QGraphicsItem *> sItems = selectedItems();

    PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();
    if (propPanel == NULL)
        qFatal("Impossible to fetch the properties panel!");

    if (sItems.count() == 1) {
        DiagramBox *selectedBox  = qgraphicsitem_cast<DiagramBox *>(sItems.at(0));
        if (selectedBox != NULL) {
            propPanel->updateBoxProperties(selectedBox);
        } else {
            qDebug() << "Non-box items selection are not yet supported";
        }
    }
}

void DiagramScene::onCancelBtnClicked(bool)
{
    QList<QGraphicsItem *> sItems = selectedItems();

    PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();
    if (propPanel == NULL)
        qFatal("Impossible to fetch the properties panel!");

    if (sItems.count() == 1) {
        DiagramBox *selectedBox  = qgraphicsitem_cast<DiagramBox *>(sItems.at(0));
        if (selectedBox != NULL) {
            // If the selected box outputs a matrix, then fetch the values for rows and cols
            if (selectedBox->outputType() == MATRIX) {
                propPanel->rowsInput()->setValue(selectedBox->rows());
                propPanel->colsInput()->setValue(selectedBox->cols());
            }
        } else {
            qDebug() << "Non-box items selection are not yet supported";
        }
    }
}
