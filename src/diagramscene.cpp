/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER

  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.

  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

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
#include "constantdiagrambox.h"
#include "addboxcommand.h"
#include "swapboxescommand.h"
#include "addlinkcommand.h"
#include "addzonecommand.h"
#include "activityfetcher.h"
#include "activityvisualizer.h"
#include "activityvisualizerbars.h"
#include "activityvisualizerthermal.h"
#include "deletelinkcommand.h"
#include "deleteboxcommand.h"
#include "deletezonecommand.h"

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
#include <QGraphicsProxyWidget>
#include <QDockWidget>
#include <QMenuBar>
#include <QCursor>
#include <QMessageBox>
#include <QInputDialog>

DiagramScene::DiagramScene(QObject *parent) : QGraphicsScene(parent),
                                            m_mainWindow(nullptr),
                                            m_leftBtnDown(false),
                                            middleBtnIsDown(false),
                                            m_rightBtnDown(false),
                                            m_shouldDrawGrid(true),
                                            m_gridSize(35),
                                            m_line(nullptr),
                                            m_rect(nullptr),
                                            m_oSlot(nullptr),
                                            m_script(nullptr),
                                            m_displayLabels(false),
                                            m_undoStack(this),
                                            m_copyGroup(nullptr)
{
	m_mainWindow = getMainWindow();

	PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();

	connect(propPanel->okBtn(), SIGNAL(clicked(bool)), this, SLOT(onOkBtnClicked(bool)));
	connect(propPanel->cancelBtn(), SIGNAL(clicked(bool)), this, SLOT(onCancelBtnClicked(bool)));
	connect(propPanel->displayVisu(), SIGNAL(clicked(bool)), this, SLOT(onDisplayVisuClicked(bool)));
	connect(this, SIGNAL(selectionChanged()), this, SLOT(onSelectionChanged()));
}

/**
 * @brief DiagramScene::~DiagramScene cleans up
 * Reminder: it has ownership of the @Script
 */
DiagramScene::~DiagramScene()
{
	if (m_line != nullptr) {
		delete m_line;
		m_line = nullptr;
	}

	if (m_rect != nullptr) {
		delete m_rect;
		m_rect = nullptr;
	}

	if (m_script != nullptr) {
		delete m_script;
		m_script = nullptr;
	}

	// The Scene will take care of deleting all items in the scene
}

void DiagramScene::addBox(DiagramBox *newBox, const QPointF &position)
{
	Q_ASSERT(newBox->outputSlot() != nullptr);

	newBox->setPos(position);

	// Add an SVG element to display to hint the size of the function (if not a constant box)
	ConstantDiagramBox *constantBox = dynamic_cast<ConstantDiagramBox *>(newBox);

	addItem(newBox);
	// If the topic name is empty, create one based on the UUID
	if (constantBox == nullptr && newBox->topic().isEmpty()) {
		newBox->setTopic(ensureSlashPrefix(mkTopicName(newBox->scriptName(), newBox->uuid().toString())));
	}
	newBox->moveBy(0.1,0); // Dirty trick to trigger the itemChange() and snap position on the grid
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

	// NOTE: we do NOT 'break' after we found a link that is invalid, because 'checkIfInvalid()'
	// will also set the link to be displayed in red when invalid. So we DO want to iterate through
	// all links because we want all invalid links to be displayed red.
	foreach (QGraphicsItem *item, allItems) {
		Link *link = dynamic_cast<Link *>(item);
		if (link != nullptr) {
			foundInvalidLinks = foundInvalidLinks || link->checkIfInvalid();
		}
	}

	// Update script's status and tab text color based on the result
	if (m_script == nullptr)
		qWarning() << "[DiagramScene::checkForInvalidLink] cannot set script as invalid: no script!";
	else
		m_script->setIsInvalid(foundInvalidLinks);

	return foundInvalidLinks;
}

bool DiagramScene::checkForInvalidity()
{
	QList<QGraphicsItem *> allItems = items();
	bool foundInvalid = false;

	// NOTE: we do NOT 'break' after we found a link that is invalid, because 'checkIfInvalid()'
	// will also set the link to be displayed in red when invalid. So we DO want to iterate through
	// all links because we want all invalid links to be displayed red.
	foreach (QGraphicsItem *item, allItems) {
		Link *link = dynamic_cast<Link *>(item);
		if (link != nullptr) {
			foundInvalid = link->checkIfInvalid() || foundInvalid;
			continue;
		}

		DiagramBox *box = dynamic_cast<DiagramBox *>(item);
		if (box != nullptr) {
			foundInvalid = box->checkIfBoxInvalid() || foundInvalid;
			continue;
		}
	}

	// Update script's status and tab text color based on the result
	if (m_script == nullptr)
		qWarning() << "[DiagramScene::checkForInvalidity] cannot set script as invalid: no script!";
	else
		m_script->setIsInvalid(foundInvalid);

	return foundInvalid;
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

		// Check if we are copying items
		if (m_copyGroup != nullptr) {
			QList<QGraphicsItem *> children = m_copyGroup->childItems();
			destroyItemGroup(m_copyGroup);
			m_copyGroup = nullptr; // Then the items won't follow the mouse anymore

			// Create a single batch command so CTRL + Z removed all copied boxes
			QUndoCommand *batchCommand = new QUndoCommand;

			// Create an AddCommand for each copied box
			foreach (QGraphicsItem *item, children) {
				DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(item);

				if (maybeBox != nullptr) {
					QPointF sP = maybeBox->scenePos();

					// Remove the item from the scene as it will be added back by the command
					maybeBox->setPos(sP);
					removeItem(maybeBox);

					// Create a new AddCommand to be chained with batchedCommand
					new AddBoxCommand(this, maybeBox, sP, batchCommand);
				}
			}

			m_undoStack.push(batchCommand);
		}

		// Check if we have clicked on something
		QGraphicsItem *maybeItem = itemAt(mousePos, QTransform());
		if (!maybeItem) {
			updateSceneRect();
		} else {
			// Check if we clicked on an output slot
			m_oSlot = dynamic_cast<OutputSlot *>(maybeItem);

			if (m_oSlot != nullptr) {
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
	} else if (evt->button() & Qt::RightButton) {
		m_rightBtnDown = true;

		// Check if we have clicked on something
		QGraphicsItem *maybeItem = itemAt(mousePos, QTransform());
		if (maybeItem) {
			updateSceneRect();
		} else {
			m_rect = new QGraphicsRectItem(evt->scenePos().x(), evt->scenePos().y(), 1, 1);
			addItem(m_rect);
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
		informUserAndCrash(tr("Only one view is supported at this moment (DiagramScene::mouseMoveEvent"));

	QWidget *viewport = vs[0]->viewport();
	QRect viewportRect(0, 0, viewport->width(), viewport->height());
	QRectF visibleArea = vs[0]->mapToScene(viewportRect).boundingRect();

	foreach(QGraphicsItem *item, items(visibleArea)) {
		Slot *slot = dynamic_cast<Slot *>(item);
		if (slot != nullptr) {
			QPointF center = slot->scenePos();
			qreal dist = (mousePos - center).manhattanLength();
			slot->setDist(dist <= 400 ? dist : 400);
			slot->update();

			// Flag input slots that can be linked to the current output slot (if any)
			InputSlot *inputSlot = dynamic_cast<InputSlot *>(slot);
			if (inputSlot != nullptr && m_leftBtnDown && m_line != nullptr && m_oSlot != nullptr) {
				inputSlot->setCanLink(canLink(m_oSlot->outputType(), inputSlot->inputType()));
			}
		}
	}

	// Draw a dotted line if we are creating a Link
	if (m_leftBtnDown && m_line != 0 && m_oSlot != nullptr) {
		QLineF newLine(m_line->line().p1(), mousePos);
		QPen currPen = m_line->pen();

		// Update line's color and thickness based on the validity of the Link
		// But don't do it for commented boxes
		InputSlot *maybeSlot = dynamic_cast<InputSlot *>(itemAt(mousePos, QTransform()));
		if (maybeSlot && !maybeSlot->box()->isCommented()) {
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

	// Draw a rectangle if the right button is pressed
	else if (m_rightBtnDown && m_rect != nullptr) {
		QRectF r(m_rect->rect());

		if (mousePos.x() > m_rect->rect().left() &&
		    mousePos.y() > m_rect->rect().top()) {
			r.setBottomRight(mousePos);
		} else if (mousePos.x() > m_rect->rect().left() &&
		           mousePos.y() < m_rect->rect().bottom()) {
			r.setTopRight(mousePos);
		} else if (mousePos.x() < m_rect->rect().right() &&
		           mousePos.y() > m_rect->rect().top()) {
			r.setBottomLeft(mousePos);
		} else if (mousePos.x() < m_rect->rect().right() &&
		           mousePos.y() < m_rect->rect().bottom()) {
			r.setTopLeft(mousePos);
		}

		m_rect->setRect(r);
	}

	// Move the copied item (have them follow the mouse) when we are copying items
	else if (m_copyGroup != nullptr) {
		// Snap the group's position's (x, y) coordinates to the grid. I don't know why, but
		// otherwise, itemChange() doesn't do its job for boxes inside the group
		// I don't understand why itemChange() doesn't fix it for these copied boxes...
		qreal newX = round(mousePos.x() / m_gridSize) * m_gridSize;
		qreal newY = round(mousePos.y() / m_gridSize) * m_gridSize;

		// Create the Point representing the new, snapped position
		QPointF snappedPos(newX, newY);

		// Remove the item from the scene as it will be added back by the command
		m_copyGroup->setPos(snappedPos);
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

			if (maybeSlot && !maybeSlot->box()->isCommented()) {
				// If we have released on top on something, check that the types are compatible
				if (canLink(m_oSlot->outputType(), maybeSlot->inputType())) {
					// And check that the link doesn't already exist
					if (!areLinked(m_oSlot, maybeSlot)) {
						// Finally, check that the slot destination is not full
						if (!isFull(maybeSlot)) {
							Link *zelda = new Link(m_oSlot, maybeSlot);
							AddLinkCommand *addLinkCommand = new AddLinkCommand(this, zelda);
//							if (m_undoStack == nullptr) {
//								qWarning() << "[DiagramScene::mouseReleaseEvent] cannot add link to scene: no undo stack!";
//								return;
//							}

							// Ask the user for the initial weight or string value upon creation
							bool isStringLink = m_oSlot->outputType() == STRING &&
							                    maybeSlot->inputType() == STRING_INPUT;

							if (isStringLink) {
								QString initialValue = QInputDialog::getText(nullptr,
								                                             tr("Link's initial value"),
								                                             tr("Value:"));
								zelda->setValue(initialValue);
							} else {
								qreal initialWeight = QInputDialog::getDouble(nullptr,
								                                              tr("Link's initial weight"),
								                                              tr("Weight:"),
								                                              1.0,
								                                              MIN_WEIGHT,
								                                              MAX_WEIGHT,
								                                              LINKS_NB_DECIMALS);

								zelda->setWeight(initialWeight);
							}

							m_undoStack.push(addLinkCommand);

							emit displayStatusMessage(tr("New link created."));
							script()->setStatusModified(true);
						} else {
							emit displayStatusMessage(tr("Slot is full: no more Link allowed!"));
						}
					} else {
						emit displayStatusMessage(tr("Link already exists!"));
					}
				} else {
					emit displayStatusMessage(tr("Invalid connection!"));
				}
			}
		}

		m_oSlot = nullptr;
	} else if (evt->button() == Qt::RightButton) {
		m_rightBtnDown = false;

		if (m_rect != nullptr) {
			QRectF r = m_rect->rect();

			Zone *z = new Zone(r.x(), r.y(), r.width(), r.height());
			AddZoneCommand *addZoneCommand = new AddZoneCommand(this, z);
//			if (m_undoStack == nullptr) {
//				qWarning() << "[DiagramScene::mouseReleaseEvent] cannot add zone to scene: no undo stack!";
//				return;
//			}
			m_undoStack.push(addZoneCommand);

			delete m_rect;
			m_rect = nullptr;
		}
	}

	updateSceneRect();

	// Important! Otherwise the box's position doesn't get updated.
	QGraphicsScene::mouseReleaseEvent(evt);
}

void DiagramScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt)
{
	Q_UNUSED(evt);
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
		// Mark boxes under cursor as drop candidates
		QList<QGraphicsView *> vs = views();
		// At the moment, for simplicity, we suppose there is only one view (that will change when
		// we introduce the minimap
		if (vs.count() != 1)
			informUserAndCrash(tr("Only one view is supported at this moment (DiagramScene::mouseMoveEvent"));

		QWidget *viewport = vs[0]->viewport();
		QRect viewportRect(0, 0, viewport->width(), viewport->height());
		QRectF visibleArea = vs[0]->mapToScene(viewportRect).boundingRect();

		// First reset all visibles boxes as not swap candidate
		foreach(QGraphicsItem *item, items(visibleArea)) {
			DiagramBox *box = dynamic_cast<DiagramBox *>(item);
			if (box != nullptr) {
				box->setSwapCandidate(false);
				box->update();
			}
		}

		// Thend check if there is a new swap candidate
		foreach (QGraphicsItem *item, items(evt->scenePos())) {
			DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(item);
			if (maybeBox != nullptr) {
				maybeBox->setSwapCandidate(true);
				maybeBox->update();
			}
		}
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
		QString iconFilepath;
		int nbInputs;
		bool constant;
		QString libname;
		MatrixShape matrixShape;
		qint32 matrixShape_;
		QString description;

		// Then proceed to retrieve the other elements
		dataStream >> name >> iconFilepath >> descriptionFile >> outputType_
		        >> constant >> nbInputs >> libname >> matrixShape_ >> description;

		// Cast the integers to the Enum type (problem of operator '>>' with enums)
		outputType = static_cast<OutputType>(outputType_);
		matrixShape = static_cast<MatrixShape>(matrixShape_);

		std::vector<InputSlot *> inputSlots;
		for (int i = 0; i < nbInputs; i += 1) {
			QString iName;
			InputType inputType;
			qint32 inputType_;
			bool multiple;
			bool checkSize;
			MatrixShape inputMatrixShape;
			qint32 inputMatrixShape_;

			dataStream >> iName >> inputType_ >> multiple >> checkSize >> inputMatrixShape_;

			// Cast the integer input types into enums
			inputType = static_cast<InputType>(inputType_);
			inputMatrixShape = static_cast<MatrixShape>(inputMatrixShape_);

			InputSlot *iSlot = new InputSlot(iName);
			iSlot->setInputType(inputType);
			iSlot->setMultiple(multiple);
			iSlot->setCheckSize(checkSize);
			iSlot->setMatrixShape(inputMatrixShape);
			inputSlots.push_back(iSlot);
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

//        DiagramBox *newBox = new DiagramBox(name, icon, outputSlot, inputSlots);
		DiagramBox *newBox;

		if (constant)
			newBox = new ConstantDiagramBox(name, outputSlot, description);
		else
			newBox = new DiagramBox(name, outputSlot, inputSlots, description);

		newBox->setDescriptionFile(descriptionFile);
		newBox->setIconFilepath(iconFilepath);
		newBox->setLibname(libname);
		newBox->setMatrixShape(matrixShape);

		// Check if we should swap boxes or not
		DiagramBox *toSwap = nullptr;
		QList<QGraphicsItem *>maybeItems = items(evt->scenePos());
		foreach (QGraphicsItem *item, maybeItems) {
			DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(item);

			if (maybeBox != nullptr) {
				toSwap = maybeBox;
				break;
			}
		}

		// If no box was found to swap, simply add the box as normal
		if (toSwap == nullptr) {
//			addBox(newBox, evt->scenePos());
			AddBoxCommand *addBoxCommand = new AddBoxCommand(this, newBox, evt->scenePos());
//			if (m_undoStack == nullptr) {
//				qWarning() << "[DiagramScene::dropEvent] cannot add box to scene: no undo stack!";
//				return;
//			}
			m_undoStack.push(addBoxCommand);

			if (m_script == nullptr)
				qWarning() << "[DiagramScene::dropEvent] cannot set script as modified: no script!";
			else
				m_script->setStatusModified(true);
			setBackgroundBrush(QBrush(Qt::white));

			QString str(tr("Function '%1' added in script").arg(name));
			emit(displayStatusMessage(str));
		} else {
			SwapBoxesCommand *swapBoxesCommand = nullptr;
			switch (QMessageBox::question(nullptr, tr("Swap boxes?"),
			                      tr("You dropped box %1 on top of box %2, do you want to swap them?")
			                              .arg(newBox->name(), toSwap->name()))) {
				case QMessageBox::Yes:
					//*
					swapBoxesCommand = new SwapBoxesCommand(this,
					                                        toSwap,
					                                        newBox);

//					if (m_undoStack == nullptr) {
//						qWarning() << "[DiagramScene::dropEvent] cannot swap boxes: no undo stack!";
//						return;
//					}
					m_undoStack.push(swapBoxesCommand);
				break;
					//*/
					// Add the new box in place of the box to swap
//					addBox(newBox, toSwap->scenePos());

					// Substitute the box information
//					newBox->setUuid(toSwap->uuid());
//					newBox->setSaveActivity(toSwap->saveActivity());
//					newBox->setPublish(toSwap->publish());
//					newBox->setTopic(toSwap->topic());
//					newBox->setTitle(toSwap->title());
					// Rows and cols are copied (valid for matrix, and ignored for scalar)
//					newBox->setRows(toSwap->rows());
//					newBox->setCols(toSwap->cols());

					// Now we transfer as much links as possible from the "old box" (toSwap)
					// to the new one. The idea is to keep all links from the inputs who have the
					// same name, and discard the others.
					// Be warned, though: same name doesn't necessarily imply same type, so the
					// copies links might be invalid.
					/*
					foreach (InputSlot *iSlot, newBox->inputSlots()) {
						if (iSlot == nullptr) {
							qWarning() << "Null pointer found in new, freshly dropped box while "
										  "trying to swap";
							continue;
						}
						// For each input of the new box, look in toSwap's inputs for similarly-named
						// This is N*M, which is ugly :/
						foreach (InputSlot *swapSlot, toSwap->inputSlots()) {
							if (swapSlot == nullptr) {
								qWarning() << "Null pointer found in box to be swapped.";
								continue;
							}

							if (swapSlot->name() != iSlot->name())
								continue;

							// We found an input slot named similarly, so now, transfer all links
							foreach (Link *link, swapSlot->inputs()) {
								// First, if this is a self-link, update the origin
								if (link->from()->box() == toSwap) {
									toSwap->outputSlot()->removeOutput(link);

									link->setFrom(newBox->outputSlot());
									newBox->outputSlot()->addOutput(link);
								}

								// Now update the destination box (should be done after, because
								// 'setTo()' checks origin box to set 'isSelfLoop'
								link->setTo(iSlot);

								// Remove this link from the inputs of the box to swap
								swapSlot->removeInput(link);

								// Add this link as inputs for the new box
								iSlot->addInput(link, true);

								// Check if this new link is invalid
								if (link->checkIfInvalid() && m_script != nullptr)
									m_script->setIsInvalid(true);
							}
						}
					}

					// Also re-link all outputs
					foreach (Link *link, toSwap->outputSlot()->outputs()) {
						if (link == nullptr) {
							qWarning() << "Null pointer found in output slot of a box to swap";
							continue;
						}

						toSwap->outputSlot()->removeOutput(link);
						newBox->outputSlot()->addOutput(link);
						link->setFrom(newBox->outputSlot());

						// Check if this new link is invalid
						if (link->checkIfInvalid() && m_script != nullptr)
							m_script->setIsInvalid(true);
					}

					// Check if the new box is invalid
					if (newBox->checkIfBoxInvalid() && m_script != nullptr)
						m_script->setIsInvalid(true);

					// Then delete swap box
					deleteItem(toSwap);
					emit displayStatusMessage(tr("Boxes swapped!"));
					//*/
//				break;

				case QMessageBox::No:
					// Mark the box as not a swap candidate anymore
					toSwap->setSwapCandidate(false);
					toSwap->update();
					emit displayStatusMessage(tr("Swapping cancelled."));
				break;

				default:
					emit displayStatusMessage(tr("Swapping cancelled."));
			}
		}

	} else {
		evt->ignore();
		emit(displayStatusMessage("Unsupported drop event, discarding."), MSG_WARNING);
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

		// If only one item is selected, delete it
		if (nbItems == 1) {
			Link *link = dynamic_cast<Link *>(items.at(0));
			DiagramBox *box = dynamic_cast<DiagramBox *>(items.at(0));
			Zone *zone = dynamic_cast<Zone *>(items.at(0));
			ActivityVisualizer *vis = dynamic_cast<ActivityVisualizer *>(items.at(0));

			if (link != nullptr)
				deleteItem(link);
			else if (box != nullptr)
				deleteItem(box);
			else if (zone != nullptr)
				deleteItem(zone);
			else if (vis != nullptr)
				delete vis;  // we don't provide CTRL + Z for deleting visualizer for now
			else
				qWarning() << "Unknown item to delete.";

			if (m_script == nullptr)
				qWarning() << "[DiagramScene::keyPressEvent] cannot set script as modified: no script!";
			else
				m_script->setStatusModified(true);
		} else if (nbItems > 0) {
			// When we have several items to delete at once, link all commands so they are only one undo/redo
			QUndoCommand *command = new QUndoCommand();

			QList<Link*> markedForDelete;

			foreach (QGraphicsItem *item, items) {
				// First, make commands for Boxes and associated Links, removing those Links
				// from the list, to prevent deleting them twice
				DiagramBox *box = dynamic_cast<DiagramBox *>(item);
				if (box != nullptr) {
					new DeleteBoxCommand(this, box, command);

					// Create delete commands for all links
					foreach (Link *outputLink, box->outputSlot()->outputs()) {
						if (!markedForDelete.contains(outputLink)) {
							new DeleteLinkCommand(this, outputLink, command);
							items.removeOne(outputLink);
							markedForDelete << outputLink;
						}
					}

					foreach (InputSlot *inputSlot, box->inputSlots()) {
						foreach (Link *inputLink, inputSlot->inputs()) {
							if (!markedForDelete.contains(inputLink)) {
								new DeleteLinkCommand(this, inputLink, command);
								items.removeOne(inputLink);
								markedForDelete << inputLink;
							}
						}
					}

					if (box->inhibInput() != nullptr) {
						foreach(Link *inhibLink, box->inhibInput()->inputs()) {
							if (!markedForDelete.contains(inhibLink)) {
								new DeleteLinkCommand(this, inhibLink, command);
								items.removeOne(inhibLink);
								markedForDelete << inhibLink;
							}
						}
					}

					items.removeOne(item);
				}
			}

			// Then re-iterate on the remaining items and create delete commands for links and zones
			foreach (QGraphicsItem *item, items) {
				Link *link = dynamic_cast<Link *>(item);
				Zone *zone = dynamic_cast<Zone *>(item);
				if (link != nullptr) {
					if (items.contains(link)) {
						new DeleteLinkCommand(this, link, command);
						items.removeOne(link);
					}
				} else if (zone != nullptr) {
					new DeleteZoneCommand(this, zone, command);
					items.removeOne(zone);
				}
			}

			// Push the commands
			m_undoStack.push(command);

			if (m_script == nullptr)
				qWarning() << "[DiagramScene::keyPressEvent] cannot set script as modified: no script!";
			else
				m_script->setStatusModified(true);

			emit displayStatusMessage(QString("Deleted %1 item%2")
			                          .arg(nbItems)
			                          .arg(nbItems != 1 ? "s" : ""));

		}

		// Also recheck for invalidity
		if (m_script == nullptr)
			qWarning() << "[DiagramScene::keyPressEvent] cannot set script as invalid: no script!";
		else
			m_script->setIsInvalid(checkForInvalidity());

	} else if (key == Qt::Key_T) {
		// Toggle displaying input slot names when 'T' is pressed
		m_displayLabels = !m_displayLabels;
		update();
	} else if (key == Qt::Key_C) {
		// Comment / decomment Function boxes
		handleComment();
	}

	QGraphicsScene::keyPressEvent(evt);
}

/**
 * @brief DiagramScene::removeItem is used to delete a @Link object from the scene
 * @param link
 */
void DiagramScene::deleteItem(Link *link)
{
	if (link == nullptr) {
		emit displayStatusMessage(tr("WARNING: tried to remove a link that was null."), MSG_WARNING);
		return;
	}

	DeleteLinkCommand *command = new DeleteLinkCommand(this, link);
	m_undoStack.push(command);

	emit displayStatusMessage(tr("Link deleted (CTRL + Z to undo)."));
}

/**
 * @brief DiagramScene::deleteItem is used to delete a box from the scene. It firsts deletes all
 * connected @Links
 * @param box
 */
void DiagramScene::deleteItem(DiagramBox *box)
{
	if (box == nullptr) {
		emit displayStatusMessage(tr("WARNING: tried to remove a box that was null."), MSG_WARNING);
		return;
	}

	DeleteBoxCommand *command = new DeleteBoxCommand(this, box);

	// Create delete commands for all links
	foreach (Link *outputLink, box->outputSlot()->outputs()) {
		new DeleteLinkCommand(this, outputLink, command);
	}

	foreach (InputSlot *inputSlot, box->inputSlots()) {
		foreach (Link *inputLink, inputSlot->inputs()) {
			new DeleteLinkCommand(this, inputLink, command);
		}
	}

	if (box->inhibInput() != nullptr) {
		foreach(Link *inhibLink, box->inhibInput()->inputs()) {
			new DeleteLinkCommand(this, inhibLink, command);
		}
	}

	m_undoStack.push(command);

	emit displayStatusMessage(tr("Function deleted (CTRL + Z to undo)."));
}

/**
 * @brief DiagramScene::deleteItem is used to delete a zone form the scene. It first un-parents all
 * boxes inside it, because we don't want to delete the boxes inside the zone when we delete it.
 * @param zone
 */
void DiagramScene::deleteItem(Zone *zone)
{
	if (zone == nullptr) {
		emit displayStatusMessage(tr("WARNING: tried to remove a zone that was null."), MSG_WARNING);
		return;
	}

	DeleteZoneCommand *command = new DeleteZoneCommand(this, zone);
	m_undoStack.push(command);

	emit displayStatusMessage(tr("Zone deleted (CTRL + Z to undo)."));
}

/**
 * @brief DiagramScene::handleComment handle commenting / decommenting Function boxes. Behaviro is
 * as such:
 * - if at least one selected box is commented (but not all), then all boxes will be commented
 * - if all selected boxes are commented, then they will all be uncommented
 * - if all selected boxes are uncommented, they will be commented
 */
void DiagramScene::handleComment()
{
	unsigned int nbCommentedBoxes = 0;
	unsigned int nbUncommentedBoxes = 0;

	// First, filter selected items to keep only (non-constant) boxes, and make us of this
	// traversal to check if we found commented and uncommented boxes
	QList<DiagramBox *> selectedBoxes;
	foreach (QGraphicsItem *item, selectedItems()) {
		DiagramBox *box = dynamic_cast<DiagramBox *>(item);
		ConstantDiagramBox *cste = dynamic_cast<ConstantDiagramBox *>(item);
		if (box == nullptr || cste != nullptr)
			continue;

		// Add this box to the list of selected boxes
		selectedBoxes << box;

		// Check if it's commented or not
		if (box->isCommented())
			nbCommentedBoxes += 1;
		else
			nbUncommentedBoxes += 1;
	}

	// Make sure we have boxes selected
	if (selectedBoxes.size() == 0 || (nbCommentedBoxes == 0 && nbUncommentedBoxes == 0))
		return;

	// Comment or uncomment
	bool commentValue;
	if (nbCommentedBoxes > 0 && nbUncommentedBoxes == 0) {
		// Uncomment all boxes
		commentValue = false;
	} else {
		// Comment all boxes
		commentValue = true;
	}

	foreach (DiagramBox *box, selectedBoxes) {
		box->setIsCommented(commentValue);
	}

	script()->setStatusModified(true);
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

QGraphicsItemGroup *DiagramScene::copyGroup() const
{
	return m_copyGroup;
}

void DiagramScene::setCopyGroup(QGraphicsItemGroup *copyGroup)
{
	m_copyGroup = copyGroup;
}

QUndoStack& DiagramScene::undoStack()
{
	return m_undoStack;
}

QGraphicsRectItem *DiagramScene::rect() const
{
	return m_rect;
}

void DiagramScene::setRect(QGraphicsRectItem *rect)
{
	m_rect = rect;
}

bool DiagramScene::rightBtnDown() const
{
	return m_rightBtnDown;
}

void DiagramScene::setRightBtnDown(bool rightBtnDown)
{
	m_rightBtnDown = rightBtnDown;
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

		DiagramBox *selectedBox = nullptr;
		Link *link = nullptr;
		Zone *zone = nullptr;

		// Display a box's, link's or comment zone properties only if there is only one selected
		if ((selectedBox = dynamic_cast<DiagramBox *>(item))) {
			propPanel->displayBoxProperties(selectedBox);
		} else if ((link = dynamic_cast<Link *>(item))) {
			propPanel->displayLinkProperties(link);
		} else if ((zone = dynamic_cast<Zone *>(item))) {
			propPanel->displayZoneProperties(zone);
		}

	} else if (sItems.count() == 0) {
		propPanel->hideBoxFrame();
		propPanel->hideLinkFrame();
		propPanel->displayScriptProperties(m_script);
	} else {
		propPanel->hideBoxFrame();
		propPanel->hideLinkFrame();
		propPanel->hideOkBtn();
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

/**
 * @brief DiagramScene::setScript sets this scene's script.
 * NOTE: the @DiagramScene takes ownership of the @Script
 * @param script the @Script to set for this scene
 */
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

/**
 * @brief DiagramScene::toggleDisplayGrid toggles the display of the grid on scenes. Note that this
 * is common to all scenes (and the state is saved in the settings)
 * @param shouldDraw whether or not the grid should be drawn
 */
void DiagramScene::toggleDisplayGrid(bool shouldDraw)
{
	m_shouldDrawGrid = shouldDraw;
	update(sceneRect());
}

/**
 * @brief DiagramScene::onOkBtnClicked validates the changes made in the properties panel. Note the
 * boolean argument is not used: it's simply here to satisfy types to bind event
 */
void DiagramScene::onOkBtnClicked(bool)
{
	// React to event only if we are the active script
	if (m_script == nullptr)
		informUserAndCrash(tr("DiagramScene doesn't have an associated script."));

	if (!m_script->isActiveScript())
		return;

	QList<QGraphicsItem *> sItems = selectedItems();

	PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();
	if (propPanel == nullptr)
		informUserAndCrash(tr("Impossible to fetch the properties panel!"));

	if (sItems.count() == 0) {
		propPanel->updateScriptProperties(m_script);
	} else if (sItems.count() == 1) {
		QGraphicsItem *item = sItems.at(0);

		DiagramBox *selectedBox;
		Link *selectedLink;
		Zone *selectedZone;

		if ((selectedBox = dynamic_cast<DiagramBox *>(item))) {
			propPanel->updateBoxProperties(selectedBox);

			// Now check all SCALAR_MATRIX links for invalidity and if there was one found, trigger
			// a recheck for the entire script
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
		} else if((selectedLink = dynamic_cast<Link *>(item))) {
			propPanel->updateLinkProperties(selectedLink);
		} else if ((selectedZone = dynamic_cast<Zone *>(item))) {
			propPanel->updateZoneProperties(selectedZone);
		} else {
			informUserAndCrash(tr("Unsupported element for updating properties, only function "
			                      "boxes, links and zones are supported at the moment."));
		}
	}

	// When we validate changes, mark the script as modified
	if (m_script != nullptr)
		m_script->setStatusModified(true);

	// After everything, update the scene to reflect changes
	update(sceneRect());
}

void DiagramScene::onCancelBtnClicked(bool)
{
	// React to event only if we are the active script
	if (m_script == nullptr)
		informUserAndCrash(tr("DiagramScene doesn't have an associated script."));

	if (!m_script->isActiveScript())
		return;

	QList<QGraphicsItem *> sItems = selectedItems();

	PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();
	if (propPanel == nullptr)
		informUserAndCrash(tr("Impossible to fetch the properties panel!"));

	// Instead of manually try to undo all changed in the properties panel, simply call
	// the displayXXX() functions which will take care of re-initializing all fields

	if (sItems.count() == 0) {
		propPanel->displayScriptProperties(m_script);
	} else if (sItems.count() == 1) {
		QGraphicsItem *item = sItems.at(0);
		DiagramBox *selectedBox;
		Link *selectedLink;
		Zone *selectedZone;

		if ((selectedBox  = dynamic_cast<DiagramBox *>(item))) {
			propPanel->displayBoxProperties(selectedBox);
		} else if ((selectedLink = dynamic_cast<Link *>(item))) {
			propPanel->displayLinkProperties(selectedLink);
		} else if ((selectedZone = dynamic_cast<Zone *>(item))) {
			propPanel->displayZoneProperties(selectedZone);
		} else {
			informUserAndCrash(tr("Unsupported element for restoring properties, only function "
			                      "boxes and links are supported at the moment."));
		}
	}
}

void DiagramScene::onDisplayVisuClicked(bool)
{
	// React to event only if we are the active script
	if (m_script == nullptr)
		informUserAndCrash(tr("DiagramScene doesn't have an associated script."));

	if (!m_script->isActiveScript())
		return;

	QList<QGraphicsItem *> sItems = selectedItems();

	PropertiesPanel *propPanel = m_mainWindow->propertiesPanel();
	if (propPanel == nullptr)
		informUserAndCrash(tr("Impossible to fetch the properties panel!"));

	if (sItems.count() == 0) {
		propPanel->setScriptTimeValue(m_script->timeValue());
		propPanel->setScriptTimeUnit(m_script->timeUnit());
	} else if (sItems.count() == 1) {
		QGraphicsItem *item = sItems.at(0);
		DiagramBox *selectedBox  = dynamic_cast<DiagramBox *>(item);
		if (selectedBox != nullptr) {
			// First check that we don't already have enabled data visualization for this box
			if (selectedBox->isActivityVisuEnabled()) {
				emit displayStatusMessage("Visualization is already enabled for this box");
				return;
			}

//			selectedBox->setIsActivityVisuEnabled(true);

			// WARNING: this is code duplication from xmlscriptreader.cpp, we should factor common code!
			ActivityVisualizer *vis = nullptr;
			switch (selectedBox->outputType()) {
				case SCALAR:
					vis = new ActivityVisualizerBars(selectedBox);
				break;

				case MATRIX:
					// (1,1) matrix is treated as a scalar
					if (selectedBox->rows() == 1 && selectedBox->cols() == 1)
						vis = new ActivityVisualizerBars(selectedBox);

					// (1,N) and (N,1) are vectors: they are displayed as several scalars
					else if (selectedBox->rows() == 1 || selectedBox->cols() == 1)
						vis = new ActivityVisualizerBars(selectedBox);

					else
						vis = new ActivityVisualizerThermal(selectedBox);
				break;

				default:
					qWarning() << "Ouput type not supported for visualization";
				return;
				break;
			}

			// Create the activity fetcher with the topic name
			ActivityFetcher *fetcher = nullptr;
			if (selectedBox->publish()) {
				fetcher = new ActivityFetcher(selectedBox->topic(), selectedBox);
			} else {
				fetcher = new ActivityFetcher(ensureSlashPrefix(mkTopicName(selectedBox->scriptName(),
				                                                            selectedBox->uuid().toString())),
				                              selectedBox);
				m_script->rosSession()->addToHotList(selectedBox->uuid());
			}

			ActivityVisualizerBars *visBar = dynamic_cast<ActivityVisualizerBars *>(vis);
			ActivityVisualizerThermal *visTh = dynamic_cast<ActivityVisualizerThermal *>(vis);
			if (visBar != nullptr) {
				visBar->setActivityFetcher(fetcher);
				connect(fetcher, SIGNAL(newMatrix(QVector<qreal>*)), visBar, SLOT(updateBars(QVector<qreal>*)));
			} else if (visTh != nullptr) {
				visTh->setActivityFetcher(fetcher);
				connect(fetcher, SIGNAL(newMatrix(QVector<qreal>*)), visTh, SLOT(updateThermal(QVector<qreal>*)));
			} else
				qWarning() << "Only Bars and Thermal visualizers are supported for now!";
//			connect(fetcher, SIGNAL(newMatrix(QVector<qreal>*)), vis, SLOT(updateMatrix(QVector<qreal>*)));

//			selectedBox->showDataVis(m_script->rosSession());

			/*
			DiagramChart *dChart = new DiagramChart(selectedBox);
			dChart->setPos(selectedBox->pos().x(), selectedBox->pos().y() - 200);
			addItem(dChart);
			//*/
		}
	}
}
