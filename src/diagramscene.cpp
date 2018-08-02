#include "diagramscene.h"
#include "librarypanel.h"
#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "slot.h"
#include "helpers.h"
#include "link.h"
#include "diagrambox.h"
#include "script.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <iostream>
#include <QKeyEvent>
#include <QList>
#include <QMimeData>
#include <QGraphicsView>
#include <QApplication>
#include <QDebug>
#include <QGraphicsSvgItem>

DiagramScene::DiagramScene(QObject *parent) : QGraphicsScene(parent),
                                            m_mainWindow(NULL),
                                            m_leftBtnDown(false),
                                            middleBtnIsDown(false),
                                            m_shouldDrawGrid(true),
                                            m_gridSize(35),
                                            m_line(NULL),
                                            m_oSlot(NULL),
                                            m_script(NULL),
                                            m_displayLabels(false)
{
    m_mainWindow = getMainWindow();

    PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();

    connect(propPanel->okBtn(), SIGNAL(clicked(bool)), this, SLOT(onOkBtnClicked(bool)));
    connect(propPanel->cancelBtn(), SIGNAL(clicked(bool)), this, SLOT(onCancelBtnClicked(bool)));
    connect(this, SIGNAL(selectionChanged()), this, SLOT(onSelectionChanged()));
}

DiagramScene::~DiagramScene()
{
    delete m_line;
    // do not call delete on m_oSlot because it's only a cast and not something we created
    delete m_script;
}

void DiagramScene::addBox(DiagramBox *newBox, const QPointF &position)
{
    Q_ASSERT(newBox->outputSlot() != NULL);

    QPointF center = newBox->boundingRect().center();
    newBox->setPos(position - center);

    // We add the Svg item here, as a child of the box, this way the box doesn't need to have knowledge of it
    QString svgPath(newBox->descriptionFile());
    svgPath.replace(".xml", ".svg");
    QGraphicsSvgItem *svg = new QGraphicsSvgItem(svgPath, newBox);

    rescaleSvgItem(svg,
                   QSizeF(newBox->bWidth() / 3 - 1.5, newBox->bHeight() - newBox->tHeight() - 2.5),
                   QPointF(newBox->bWidth() / 3, 1.5));

    // Add an SVG element to display to hint the size of the function
    svgPath = ":/icons/icons/size-icon.svg";
    QGraphicsSvgItem *s = new QGraphicsSvgItem(svgPath, newBox);
    newBox->setSizeIcon(s);
    updateSizeIcon(newBox);

    rescaleSvgItem(s,
                   QSizeF(newBox->bWidth() / 3 - 1.5, newBox->bHeight() - newBox->tHeight() - 2.5),
                   QPointF(2 * newBox->bWidth() / 3, 1.5));

    addItem(newBox);
}

/**
 * @brief DiagramScene::checkForInvalidLinks makes a global on <b>all</b> items on the scene, filter
 * by Link and check every one of them for invalidity.
 * This is costly, so it should be done only when necessary
 * @return
 */
bool DiagramScene::checkForInvalidLinks()
{
    QList<QGraphicsItem *> allItems = items();
    bool foundInvalidLinks = false;

    foreach (QGraphicsItem *item, allItems) {
        Link *link = dynamic_cast<Link *>(item);
        if (link != NULL) {
            foundInvalidLinks |= link->checkIfInvalid();
        }
    }

    // Update script's status and tab text color based on the result
    m_script->setIsInvalid(foundInvalidLinks);

    return foundInvalidLinks;
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
        QSize widgetSize = m_mainWindow->ui()->tabWidget->size();

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
    QPointF mousePos = evt->scenePos();

    if (evt->button() & Qt::LeftButton) {
        m_leftBtnDown = true;

        // Check if we have clicked on something
        QGraphicsItem *maybeItem = itemAt(mousePos, QTransform());
        if (!maybeItem) {
            updateSceneRect();
        } else {
            // Check if we clicked on an output slot
            m_oSlot = dynamic_cast<OutputSlot *>(maybeItem);

            if (m_oSlot != NULL) {
                // Display input slot's names when creating a link
                m_prevDisplayLabels = m_displayLabels;
                m_displayLabels = true;

                QPointF startPoint = m_oSlot->scenePos();
                m_line = new QGraphicsLineItem(QLineF(startPoint, mousePos));
                m_line->setPen(QPen(Qt::black, 1, Qt::DashLine));
                addItem(m_line);
                updateSceneRect();
            }
        }
    }

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
    QPointF mousePos = evt->scenePos();

    if (evt->button() == Qt::LeftButton) {
        m_leftBtnDown = false;

        // Remove temporary line if we were drawing one (click initiated on an output slot)
        if (m_line != 0) {
            // Deleting the line is enough, the QGraphicsScene will take care of removing it
            delete m_line;
            m_line = 0;

            // Restore the state of names display when releasing mouse if we were creating a Link
            m_displayLabels = m_prevDisplayLabels;
            update();

            // Check if we have released on top of an input slot and create a Link if so
            InputSlot *maybeSlot = dynamic_cast<InputSlot *>(itemAt(mousePos, QTransform()));

            if (maybeSlot) {
                // If we have released on top on something, check that the types are compatible
                if (canLink(m_oSlot->outputType(), maybeSlot->inputType())) {
                    // And check that the link doesn't already exists
                    if (!areLinked(m_oSlot, maybeSlot)) {
                        Link *zelda = new Link(m_oSlot, maybeSlot);
                        addItem(zelda); // Important to add the Link to the scene first
                        zelda->addLinesToScene(); // And then it's important to call this to add the segments to the scene
                        if (zelda->checkIfInvalid()) {
                            emit displayStatusMessage(tr("Warning: sizes do not correspond!"));
                            script()->setIsInvalid(true);
                        }

                        zelda->setZValue(LINKS_Z_VALUE);

                        emit displayStatusMessage(tr("New link created."));
                        script()->setStatusModified(true);
                    } else {
                        emit displayStatusMessage(tr("Link already exists!"));
                    }
                } else {
                    emit displayStatusMessage(tr("Invalid connection!"));
                }
            }
        }

        m_oSlot = 0;
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
        emit displayStatusMessage(tr("Unknown MIME type received: ignoring."));
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
        OutputType outputType;
        qint32 outputType_;
        QIcon icon;
        int nbInputs;
        bool constant;

        // Then proceed to retrieve the other elements
        dataStream >> name >> icon >> descriptionFile >> outputType_ >> constant >> nbInputs;

        // Cast the integer to the Enum type (problem of operator '>>' with enums)
        outputType = static_cast<OutputType>(outputType_);

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

        OutputSlot *outputSlot = new OutputSlot;
        outputSlot->setOutputType(outputType);

        /*
        // TODO: join this with the previous foreach?
        std::set<InputSlot *> inputSlots;
        foreach (QString iName, inputNames) {
            InputSlot *iSlot = new InputSlot(iName);
            inputSlots.insert(iSlot);
        }
        //*/

        DiagramBox *newBox = new DiagramBox(name, icon, outputSlot, inputSlots);
        newBox->setConstant(constant);
        newBox->setDescriptionFile(descriptionFile);
        addBox(newBox, evt->scenePos());
        m_script->setStatusModified(true);

        setBackgroundBrush(QBrush(Qt::white));
        QString str(tr("Function '%1' added in script").arg(name));
        emit(displayStatusMessage(str));
    } else {
        evt->ignore();
        emit(displayStatusMessage("Unsupported drop event, discarding."));
    }
}

/**
 * @brief DiagramScene::keyPressEvent handles key pressed
 * - DEL: deletes the selected item(s)
 * - T  : toggles displaying input slot's text labels
 * @param evt
 */
void DiagramScene::keyPressEvent(QKeyEvent *evt)
{
    int key = evt->key();

    // Delete selected items when 'DELETE' is pressed
    if (key == Qt::Key_Delete) {
        QList<QGraphicsItem *> items = selectedItems();
        int nbItems = items.count();

        foreach (QGraphicsItem *item, items) {
            Link *link = dynamic_cast<Link *>(item);
            if (link != NULL) {
                deleteItem(link);
                continue;
            }

            DiagramBox *box = dynamic_cast<DiagramBox *>(item);
            if (box != NULL) {
                deleteItem(box);
                continue;
            }
        }
        /*
        for (int i = 0; i < nbItems; i += 1) {
            // Remove the item
            removeItem(qgraphicsitem_cast<DiagramBox *>(items.at(i)));
            updateSceneRect();
        }
        //*/

        // Set the associated script as modified if there was a deletion
        if (nbItems > 0) {
            m_script->setStatusModified(true);
            emit displayStatusMessage(tr("Deleted ") + nbItems + " items.");
        }
    } else if (key == Qt::Key_T) {
        // Toggle displaying input slot names when 'T' is pressed
        m_displayLabels = !m_displayLabels;
        update();
    }

    QGraphicsScene::keyPressEvent(evt);
}

/**
 * @brief DiagramScene::removeItem is used to delete a @Link object from the scene
 * @param link
 */
void DiagramScene::deleteItem(Link *link)
{
    if (link == NULL) {
        emit displayStatusMessage(tr("WARNING: tried to remove a link that was null."));
        return;
    }

    // First, remove this link from its OutputSlot
    if (link->from() != NULL) {
        link->from()->removeOutput(link);
    } else {
        emit displayStatusMessage(tr("WARNING: tried to remove a link that did not have an "
                                     "originating output slot."));
    }

    // Then, remove this link from its InputSlot
    if (link->to() != NULL) {
        link->to()->removeInput(link);
    } else {
        emit displayStatusMessage(tr("WARNING: tried to remove a link that did not have an ending "
                                     "input slot."));
    }

    // And finally, delete the Link (the QGraphicsScene will take care of removing the object)
    delete link;
}

/**
 * @brief DiagramScene::deleteItem is used to delete a box from the scene. It firsts deletes all
 * connected @Links
 * @param box
 */
void DiagramScene::deleteItem(DiagramBox *box)
{
    if (box == NULL) {
        emit displayStatusMessage(tr("WARNING: tried to remove a box that was null."));
        return;
    }

    // First, delete all links attached to this box
    foreach (Link *outputLink, box->outputSlot()->outputs()) {
        deleteItem(outputLink);
    }

    foreach (InputSlot *inputSlot, box->inputSlots()) {
        foreach (Link *inputLink, inputSlot->inputs()) {
            deleteItem(inputLink);
        }
    }

    // Finally, delete the box (the QGraphicsScene will take care of removing the box)
    delete box;
}

void DiagramScene::removeItem(QGraphicsItem *item)
{
    QGraphicsScene::removeItem(item);
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

bool DiagramScene::displayLabels() const
{
    return m_displayLabels;
}

void DiagramScene::setDisplayLabels(bool displayLabels)
{
    m_displayLabels = displayLabels;
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

    if (sItems.count() == 1) {
        QGraphicsItem *item = sItems.at(0);

        // Display a box's or link's properties only if there is only one selected
        DiagramBox *selectedBox  = dynamic_cast<DiagramBox *>(item);
        if (selectedBox != NULL) {
            propPanel->displayBoxProperties(selectedBox);
        } else {
            Link *link = dynamic_cast<Link *>(item);
            if (link != NULL) {
                propPanel->displayLinkProperties(link);
            } else {
                emit displayStatusMessage(tr("Only function box or link can have their properties displayed!"));
            }
        }
    } else if (sItems.count() == 0) {
        propPanel->boxFrame()->hide();
        propPanel->linkFrame()->hide();
        propPanel->displayScriptProperties(m_script);
    } else {
        propPanel->boxFrame()->hide();
        propPanel->linkFrame()->hide();
        propPanel->okBtn()->hide();
        propPanel->cancelBtn()->hide();
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
        informUserAndCrash(tr("Impossible to fetch the properties panel!"));

    if (sItems.count() == 0) {
        propPanel->updateScriptProperties(m_script);
    } else if (sItems.count() == 1) {
        QGraphicsItem *item = sItems.at(0);

        DiagramBox *selectedBox  = dynamic_cast<DiagramBox *>(item);
        if (selectedBox != NULL) {
            propPanel->updateBoxProperties(selectedBox);

            // Now check all SCALAR_MATRIX links for invalidity and if there was one found, trigger
            // a recheck for the entie script
            bool linkWasFixed = false;   // wether a link switched from invalid to valid
            bool invalidLinkFound = false; // wether we found an invalid link

            foreach (InputSlot *inputSlot, selectedBox->inputSlots()) {
                if (inputSlot->inputType() == SCALAR_MATRIX) {
                    // Recheck every of these links individually
                    foreach (Link *link, inputSlot->inputs()) {
                        bool wasInvalid = link->isInvalid();
                        bool isInvalidNow = link->checkIfInvalid();

                        if (isInvalidNow)
                            invalidLinkFound = true;

                        if (wasInvalid && !isInvalidNow) {
                            linkWasFixed = true;
                        }
                    }
                }
            }

            // Do the same for output slots
            foreach (Link *link, selectedBox->outputSlot()->outputs()) {
                bool wasInvalid = link->isInvalid();
                bool isInvalidNow = link->checkIfInvalid();

                if (isInvalidNow)
                    invalidLinkFound = true;

                if (wasInvalid && !isInvalidNow)
                    linkWasFixed = true;
            }

            // After checking all individual links, decide if we need to trigger a global recheck
            if (!invalidLinkFound && linkWasFixed) {
                emit displayStatusMessage(tr("Initiating global recheck for invalid link..."));
                if (checkForInvalidLinks())
                    emit displayStatusMessage(tr("Invalid links were found: script is in invalid state!"));
                else
                    emit displayStatusMessage(tr("Everything's ok, no invalid links were found."));
            }
            else if (invalidLinkFound) {
                emit displayStatusMessage(tr("Invalid link found!"));
                script()->setIsInvalid(true);
            }
        } else {
            Link *selectedLink = dynamic_cast<Link *>(item);
            if (selectedLink != NULL) {
                propPanel->updateLinkProperties(selectedLink);
            } else {
                informUserAndCrash(tr("Unsupported element for updating properties, only function "
                                      "boxes and links are supported at the moment."));
            }
        }
    }
}

void DiagramScene::onCancelBtnClicked(bool)
{
    QList<QGraphicsItem *> sItems = selectedItems();

    PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();
    if (propPanel == NULL)
        informUserAndCrash(tr("Impossible to fetch the properties panel!"));

    if (sItems.count() == 0) {
        propPanel->timeValue()->setValue(m_script->timeValue());
        propPanel->timeUnit()->setCurrentIndex(m_script->timeUnit());
    } else if (sItems.count() == 1) {
        QGraphicsItem *item = sItems.at(0);
        DiagramBox *selectedBox  = dynamic_cast<DiagramBox *>(item);
        if (selectedBox != NULL) {
            // If the selected box outputs a matrix, then fetch the values for rows and cols
            if (selectedBox->outputType() == MATRIX) {
                propPanel->rowsInput()->setValue(selectedBox->rows());
                propPanel->colsInput()->setValue(selectedBox->cols());
            }
        } else {
            Link *selectedLink = dynamic_cast<Link *>(item);
            if (selectedLink != NULL) {
                propPanel->linkWeight()->setValue(selectedLink->weight());
            } else {
                informUserAndCrash(tr("Unsupported element for restoring properties, only function "
                                      "boxes and links are supported at the moment."));
            }
        }
    }
}