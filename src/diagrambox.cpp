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

#include "diagrambox.h"
#include "diagramscene.h"
#include "papyruswindow.h"
#include "helpers.h"
#include "constants.h"
#include "scalarvisualization.h"
//#include "vectorvisualization.h"
#include "matrixvisualization.h"
#include "zone.h"
#include "movecommand.h"

#include <iostream>

#include <QApplication>
#include <QPen>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <math.h>
#include <QDebug>
#include <QGraphicsLayout>

Q_DECLARE_METATYPE(MatrixShape);

int DiagramBox::getType()
{
	return UserType + 1;
}

DiagramBox::DiagramBox(const QString &name,
                       const QIcon &icon,
                       const QString &description,
                       OutputSlot *outputSlot,
                       std::vector<InputSlot *> inputSlots,
                       const QUuid &uuid,
                       QGraphicsItem *parent) : QGraphicsItem(parent),
                                                m_name(name),
                                                m_bWidth(120),
                                                m_bHeight(70),
                                                m_tHeight(20),
                                                m_matrixShape(SHAPE_NONE),
                                                m_uuid(uuid),
                                                m_icon(icon),
                                                m_description(description),
                                                m_outputSlot(outputSlot),
                                                m_inputSlots(inputSlots),
                                                m_rows(1),
                                                m_cols(1),
                                                m_saveActivity(false),
                                                m_publish(false),
                                                m_sizeIcon(nullptr),
                                                m_dataVis(nullptr),
                                                m_dataProxy(nullptr),
                                                m_isInvalid(false),
                                                m_swapCandidate(false)
{
	// Generate a UUID if there was not one while created
	if (m_uuid.isNull())
		m_uuid = QUuid::createUuid();

	setFlags(QGraphicsItem::ItemIsSelectable
	       | QGraphicsItem::ItemIsMovable
	       | QGraphicsItem::ItemSendsScenePositionChanges);
	setAcceptHoverEvents(true);

	setToolTip(description);
	// Make this the parent item of the output slot, so that it follow drags, etc.
	m_outputSlot->setParentItem(this);
	m_outputSlot->setBox(this);
	m_outputSlot->setAcceptHoverEvents(true);

	// Set the output's slot position, in its parent's referential (this item's)
	setOutputSlotPos();

	// Make this the parent item of all input slots, so that they follow drags, etc.
	qreal s = 20;
	qreal offset = m_inputSlots.size() % 2 == 0 ? s / 2 : 0; // offset only if even nb of slots

	QPointF g = (boundingRect().bottomLeft() + boundingRect().topLeft()) / 2;
	g.rx() -= 5;

	g.ry() -= ((m_inputSlots.size() - 1) / 2) * s + offset;

	// Get the PropertiesPanel and connect its display slot to this box's signal
	PapyrusWindow *mainWindow = NULL;

	foreach (QWidget *w, qApp->topLevelWidgets()) {
		if (PapyrusWindow *mW = qobject_cast<PapyrusWindow *>(w)) {
			mainWindow = mW;
			break;
		}
	}

	if (mainWindow) {
		PropertiesPanel *propertiesPanel = mainWindow->propertiesPanel();

		if (propertiesPanel == NULL)
			qFatal("PropertiesPanel doesn't exist!");

		connect(this, SIGNAL(boxSelected(DiagramBox *)), propertiesPanel, SLOT(displayBoxProperties(DiagramBox *)));
	}

	foreach (InputSlot *inputSlot, m_inputSlots) {
		inputSlot->setParentItem(this);
		inputSlot->setBox(this);
		inputSlot->setAcceptHoverEvents(true);

		/*
		 * How we compute the (vertical) positions:
		 * - we want the inputs slots to be evenly spaced and centered on the box's center
		 * - we define 's' the distance between each input slots' center
		 * - we use integer division to correctly place the input slots
		 * - in case there are an even number of inputs, we add `s/2` as offset
		 */
		inputSlot->setPos(g);
		g.ry() += s;
	}

	setZValue(BOXES_Z_VALUE);
}

DiagramBox::~DiagramBox()
{
	// Normally, upon deletion, the box should not have any remaining Link connected
	if (!m_outputSlot->outputs().empty())
		qWarning() << "WARNING: DiagramBox(" << m_name << ") is being destructed, but there remains Links on its OutputSlot";
	delete m_outputSlot;

	bool warnedForInput = false;
	foreach (InputSlot *inputSlot, m_inputSlots) {
		if (!warnedForInput && !inputSlot->inputs().empty()) {
			warnedForInput = true;
			qWarning() << "WARNING: DiagramBox(" << m_name << ") is being destructed, but there remains Links on its InputSlot (" << inputSlot->name() << ")";
		}
		delete inputSlot;
	}
	m_inputSlots.clear();

	delete m_sizeIcon;
}

QRectF DiagramBox::boundingRect() const
{
	// +7 are there to include the publish logo on the bottom right
	return QRectF(0, 0, m_bWidth + 7, m_bHeight + 7);
}

/*
 * React to when the DiagramBox experiences a change
 * This is where we handle operations where a box is moved (dragged) on the scene: move its
 * connected links, etc.
 */
QVariant DiagramBox::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
	// When it is moved, we need to move its connected Links
	if (change == QGraphicsItem::ItemPositionChange && scene()) {
		// Get coordinate of the target new position
		QPointF targetPos = value.toPointF();

		// Get the scene in order to get the grid size
		DiagramScene *theScene = dynamic_cast<DiagramScene *>(scene());
		if (theScene == NULL) {
			informUserAndCrash("Could not cast the scene into a DiagramScene!");
		}
		int gridSize = theScene->gridSize();

		// Snap the new position's (x, y) coordinates to the grid
		qreal newX = round(targetPos.x() / gridSize) * gridSize;
		qreal newY = round(targetPos.y() / gridSize) * gridSize;

		// Create the Point representing the new, snapped position
		QPointF newPos(newX, newY);

		// Prompt the output slot and all inputs slots to update their connected links
		m_outputSlot->updateLinks();
		foreach (InputSlot *inputSlot, m_inputSlots) {
			inputSlot->updateLinks();
		}

		// Set the script to which this item's scene is associated as modified
		theScene->script()->setStatusModified(true);

		return newPos;
	}

	return QGraphicsItem::itemChange(change, value);
}

bool DiagramBox::swapCandidate() const
{
	return m_swapCandidate;
}

void DiagramBox::setSwapCandidate(bool swapCandidate)
{
	m_swapCandidate = swapCandidate;
}

void DiagramBox::setUuid(const QUuid &uuid)
{
	m_uuid = uuid;
}

BoxInvalidReason DiagramBox::invalidReason() const
{
	return m_invalidReason;
}

void DiagramBox::setInvalidReason(const BoxInvalidReason &invalidReason)
{
	m_invalidReason = invalidReason;
}

bool DiagramBox::isInvalid() const
{
	return m_isInvalid;
}

void DiagramBox::setIsInvalid(bool isInvalid)
{
	m_isInvalid = isInvalid;
}

MatrixShape DiagramBox::matrixShape() const
{
	return m_matrixShape;
}

void DiagramBox::setMatrixShape(const MatrixShape &matrixShape)
{
	m_matrixShape = matrixShape;

	// Also set the corresponding fields to 1 when appropriate
	if (matrixShape == POINT) {
		m_rows = 1;
		m_cols = 1;
	} else if (matrixShape == ROW_VECT) {
		m_rows = 1;
	} else if (matrixShape == COL_VECT) {
		m_cols = 1;
	}
}

QString DiagramBox::title() const
{
	return m_title;
}

void DiagramBox::setTitle(const QString &title)
{
	m_title = title;
}

void DiagramBox::onDataVisClosed()
{
	delete m_dataProxy;    // also deletes the widget
	m_dataProxy = nullptr;
	m_dataVis = nullptr;
}

DataVisualization *DiagramBox::dataVis() const
{
	return m_dataVis;
}

void DiagramBox::setDataVis(DataVisualization *dataVis)
{
	m_dataVis = dataVis;
}

QString DiagramBox::libname() const
{
	return m_libname;
}

void DiagramBox::setLibname(const QString &libname)
{
	m_libname = libname;
}

QString DiagramBox::iconFilepath() const
{
	return m_iconFilepath;
}

void DiagramBox::setIconFilepath(const QString &value)
{
	m_iconFilepath = value;
}

QString DiagramBox::topic() const
{
	return m_topic;
}

void DiagramBox::setTopic(const QString &topic)
{
	m_topic = topic;
}

bool DiagramBox::publish() const
{
	return m_publish;
}

void DiagramBox::setPublish(bool publish)
{
	m_publish = publish;
}

QGraphicsSvgItem *DiagramBox::sizeIcon() const
{
	return m_sizeIcon;
}

void DiagramBox::setSizeIcon(QGraphicsSvgItem *sizeIcon)
{
	m_sizeIcon = sizeIcon;
}

qreal DiagramBox::tHeight() const
{
	return m_tHeight;
}

qreal DiagramBox::bHeight() const
{
	return m_bHeight;
}

qreal DiagramBox::bWidth() const
{
	return m_bWidth;
}

bool DiagramBox::saveActivity() const
{
	return m_saveActivity;
}

void DiagramBox::setSaveActivity(bool saveActivity)
{
	m_saveActivity = saveActivity;
}

int DiagramBox::cols() const
{
	return m_cols;
}

void DiagramBox::setCols(int cols)
{
	if (cols < 0)
		qFatal("Cannot have a negative number of columns");

	m_cols = cols;
}

int DiagramBox::rows() const
{
	return m_rows;
}

void DiagramBox::setRows(int rows)
{
	if (rows < 0)
		qFatal("Cannot have a negative number of rows");

	m_rows = rows;
}

/**
 * @brief DiagramBox::outputType returns this function's output slot's output type, re-implemented
 * for convenience.
 * @return
 */
OutputType DiagramBox::outputType() const
{
	return m_outputSlot->outputType();
}

/**
 * @brief DiagramBox::setOutputType sets this function's output slot's output type, re-implemented
 * for convenience
 * @param outputType
 */
void DiagramBox::setOutputType(const OutputType &outputType)
{
	m_outputSlot->setOutputType(outputType);
}

std::vector<InputSlot *> DiagramBox::inputSlots() const
{
	return m_inputSlots;
}

void DiagramBox::setInputSlots(const std::vector<InputSlot *> &inputSlots)
{
	m_inputSlots = inputSlots;
}

OutputSlot *DiagramBox::outputSlot() const
{
	return m_outputSlot;
}

void DiagramBox::setOutputSlot(OutputSlot *outputSlot)
{
	qDebug() << "SET OUTPUT SLOT";
	m_outputSlot = outputSlot;
}

QIcon DiagramBox::icon() const
{
	return m_icon;
}

void DiagramBox::setIcon(const QIcon &icon)
{
	m_icon = icon;
}

QString DiagramBox::descriptionFile() const
{
	return m_descriptionFile;
}

void DiagramBox::setDescriptionFile(const QString &descriptionFile)
{
	m_descriptionFile = descriptionFile;
}

QUuid DiagramBox::uuid() const
{
	return m_uuid;
}

int DiagramBox::type()
{
	return DiagramBox::getType();
}

void DiagramBox::setName(const QString &name)
{
	m_name = name;
}

QString DiagramBox::name() const
{
	return m_name;
}

QString DiagramBox::description() const
{
	return m_description;
}

void DiagramBox::setDescription(const QString &description)
{
	m_description = description;
}

/**
 * @brief DiagramBox::paint draw the function box: its shape, bold when selected, function's icon,
 * function's name, etc.
 * @param painter
 * @param option
 * @param widget
 */
void DiagramBox::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(widget);

	QPen pen;
	qreal width = 1.5;

	QFont font = painter->font();
	font.setPixelSize(13);

	// If the box is selected, make it appear bold
	if (option->state & QStyle::State_Selected) {
		width += 1;
		font.setBold(true);
	}

	pen.setWidthF(width);
	painter->setFont(font);

	QColor color = Qt::gray;
	color = color.dark();

	if (m_isInvalid)
		color = Qt::red;

	pen.setColor(color);

	painter->setPen(pen);

	// We have to calculate offsets to draw lines in order to stay inside the boundingRect
	qreal x0 = 0 + width / 2.0;
	qreal y0 = x0;
	qreal w = m_bWidth - width; // width/2 on the left and width/2 on the right, hence width
	qreal h = m_bHeight - width;

	// Draw enclosure
	painter->drawRoundedRect(QRectF(x0, y0, w, h), 7, 7);
	// Fill enclosure (use a QPainterPath because there's no fillRoundedRect()
	QPainterPath encPath;
	encPath.addRoundedRect(QRectF(x0, y0, w, h), 7, 7);

	// Draw a little arrow when the box is set to publish its output
	if (m_publish) {
		// Save pen width
		qreal origPenWidth = pen.widthF();
		QColor origPenColor = pen.color();
		pen.setWidthF(2.0);
		pen.setColor(QColor(0, 128, 128));
		painter->setPen(pen);

		// Arc
		qreal rectStartX = m_bWidth - 15;
		qreal rectStartY = m_bHeight - 15;
		qreal rectWidth   = 17;
		qreal rectHeight  = 17;

		QRectF arcRec(rectStartX,
		              rectStartY,
		              rectWidth,
		              rectHeight);

		int startAngle = 0 * 16;
		int stopAngle  = -90 * 16;

		painter->drawArc(arcRec, startAngle, stopAngle);
		arcRec.adjust(-4, -4, 4, 4);
		painter->drawArc(arcRec, startAngle, stopAngle);

		// Restore pen's width
		pen.setWidthF(origPenWidth);
		pen.setColor(origPenColor);
		painter->setPen(pen);
	}

	// Hint functions that are swap candidates
	if (m_swapCandidate) {
		QColor c(153, 102, 255, 100);
		painter->fillPath(encPath, QBrush(c));
	} else
		painter->fillPath(encPath, Qt::white);

	// Draw vertical lines to create compartments
	painter->drawLine(QLineF(m_bWidth / 2.0 - width / 2.0, 1.5 * width, m_bWidth / 2.0 - width / 2.0, m_bHeight - m_tHeight - width));

	// Draw horizontal line to create the space for the function's name, with dashed line
	pen.setStyle(Qt::DotLine);
	painter->setPen(pen);

	// width and not 1.5 * width at the end for aesthetics (dots don't go toward the end of line)
	painter->drawLine(QLineF(1.5 * width, m_bHeight - m_tHeight, m_bWidth - width, m_bHeight - m_tHeight));

	pen.setStyle(Qt::SolidLine);
	painter->setPen(pen);

	// The function's icon is not drawn here since it's a SVG element set as a child of this one

	// Draw the function's name or title if present
	QString toDisplay = m_name;
	if (!m_title.isEmpty())
		toDisplay = m_title;

	painter->drawText(QRectF(0, m_bHeight - m_tHeight, m_bWidth, m_tHeight), Qt::AlignCenter, toDisplay);
}

void DiagramBox::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
	QGraphicsItem::mouseDoubleClickEvent(event);
}

void DiagramBox::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
	Q_UNUSED(evt);

	m_oldPos = scenePos();
}

/**
 * @brief DiagramBox::mouseReleaseEvent check if this @DiagramBox was moved _outside_ a @Zone, and
 * if yes, it removes itself from this @Zone's children
 * @param event
 */
void DiagramBox::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	QPointF newPos = scenePos();
	bool moved = (m_oldPos != newPos);

	QList<QGraphicsItem *> colliding = collidingItems();
	bool onZone = false;

	foreach (QGraphicsItem *item, colliding) {
		Zone *z = dynamic_cast<Zone *>(item);

		if (z != nullptr) {
			onZone = true;
			break;
		}
	}

	// If the box had been dropped outside a zone, make sure we don't have a parent anymore
	if (!onZone && parentItem() != nullptr) {
		QPointF savedPos = parentItem()->mapToScene(pos());
		setParentItem(nullptr);
		setPos(savedPos);

		// And then update the displaying of its links because otherwise they go back to pointing
		// to some weird location
		if (outputSlot() != nullptr)
			outputSlot()->updateLinks();

		foreach (InputSlot *iSlot, inputSlots()) {
			iSlot->updateLinks();
		}
	}

	if (moved) {
		MoveCommand *moveCommand = new MoveCommand(this, m_oldPos);

		DiagramScene *dScene = dynamic_cast<DiagramScene *>(scene());
		if (dScene == nullptr) {
			qWarning() << "[DiagramBox::mouseReleaseEvent] cannot add MoveCommand to undo stack: no "
			              "scene found!";
			return QGraphicsItem::mouseReleaseEvent(event);
		}

		dScene->undoStack()->push(moveCommand);
	}

	QGraphicsItem::mouseReleaseEvent(event);
}

/**
 * @brief DiagramBox::getScript returns a pointer to the @Script object, if this box has been added
 * to a scene and this scene has a script. Returns NULL otherwise.
 */
Script *DiagramBox::getScript()
{
	DiagramScene *dScene = dynamic_cast<DiagramScene *>(scene());

	if (dScene == nullptr)
		return nullptr;

	return dScene->script();
}

/**
 * @brief DiagramBox::checkIfBoxInvalid checks if the DiagramBox is invalid. Reasons to be invalid are:
 * - if one of it input slots that have 'multiple="false"' has several inputs connected
 */
bool DiagramBox::checkIfBoxInvalid()
{
	// Begin with a valid box
	m_isInvalid = false;
	m_invalidReason = INVALID_BOX_INVALID_REASON;

	foreach (InputSlot *iSlot, inputSlots()) {
		if (iSlot == nullptr) {
			qWarning() << "Null pointer inside input slots";
			continue;
		}

		// Ignore slots that are authorized to have several inputs
		if (iSlot->multiple())
			continue;

		if (iSlot->inputs().size() > 1) {
			m_isInvalid = true;
			m_invalidReason = m_invalidReason | INPUT_FULL;

			break; // No need to check other inputs: we have found an invalid reason
		}
	}

	// Check the dimensions against the shape
	if (m_matrixShape == POINT && (m_rows != 1 || m_cols != 1)) {
		m_isInvalid = true;
		m_invalidReason = m_invalidReason | BOX_MUST_BE_POINT;
	} else if (m_matrixShape == VECT && m_rows != 1 && m_cols != 1) {
		m_isInvalid = true;
		m_invalidReason = m_invalidReason | BOX_MUST_BE_VECT;
	} else if (m_matrixShape == ROW_VECT && m_rows != 1) {
		m_isInvalid = true;
		m_invalidReason = m_invalidReason | BOX_MUST_BE_ROW_VECT;
	} else if (m_matrixShape == COL_VECT && m_cols != 1) {
		m_isInvalid = true;
		m_invalidReason = m_invalidReason | BOX_MUST_BE_COL_VECT;
	}

	updateTooltip();
	return m_isInvalid;
}

/**
 * @brief DiagramBox::updateTooltip update the tooltip of the box, based on the reason why it's
 * invalid. Reset the tooltip to description string if valid
 */
void DiagramBox::updateTooltip()
{
	QString str;

	if (m_invalidReason & INPUT_FULL)
		str += tr("<li>some input with <pre>multiple = \"false\"</pre> have several connected links</li>");

	if (m_invalidReason & BOX_MUST_BE_POINT)
		str += tr("<li>this box must be a point (a (1,1) matrix)</li>");
	else if (m_invalidReason & BOX_MUST_BE_VECT)
		str += tr("<li>this box must be a vector (either a (1, N) or (N, 1) matrix)</li>");
	else if (m_invalidReason & BOX_MUST_BE_ROW_VECT)
		str += tr("<li>this box must be a row vector (a (1, N) matrix)</li>");
	else if (m_invalidReason & BOX_MUST_BE_COL_VECT)
		str += tr("<li>this box must be a column vector (a (N, 1) matrix)</li>");

	if (!str.isEmpty())
		str = tr("Box is <strong>invalid</strong>:<ul>") + str + "</ul>";
	else
		str = description();

	setToolTip(str);
}

void DiagramBox::showDataVis(ROSSession *rosSession)
{
	// First check that there isn't already a visualization window, otherwise do nothing
	if (m_dataVis != nullptr)
		return;

	// Otherwise, create the DataVisualization and add it to the scene
//	m_dataVis = new DataVisualization(nullptr, scene(), this);

	QString winTitle = "Visualization";

	switch (outputType()) {

		case SCALAR:
			m_dataVis = new ScalarVisualization(nullptr, rosSession, scene(), this);
			winTitle = "Scalar visualization";
		break;

		case MATRIX:
			// Discriminate between (1,1) (treat as scalar), rows and cols
			if (m_rows == 1 && m_cols == 1) {
				m_dataVis = new ScalarVisualization(nullptr, rosSession, scene(), this);
				winTitle = "Scalar visualization";
			}
			else if (m_rows == 1 || m_cols == 1) {
				m_dataVis = new ScalarVisualization(nullptr, rosSession, scene(), this);
				winTitle = "Scalar visualization";
			}
			else {
				m_dataVis = new MatrixVisualization(nullptr, rosSession, scene(), this);
				winTitle = "Matrix visualization";
			}
		break;

		default:
			qDebug() << "Ouput type not supported for visualization";
		    return;
		break;
	}

	m_dataProxy = scene()->addWidget(m_dataVis, Qt::Window);
	m_dataProxy->setWindowTitle(winTitle);
	m_dataProxy->setGeometry(QRectF(0, 0, 400, 400 / 1.618));

	connect(m_dataProxy, SIGNAL(visibleChanged()), this, SLOT(onDataVisClosed()));

	QPointF p = scenePos();

	p.ry() -= (400 / 1.618); // to place the visu window above the box
	m_dataProxy->setPos(p);
	m_dataProxy->setFlags(QGraphicsItem::ItemIsSelectable
	                      | QGraphicsItem::ItemIsMovable
	                      | QGraphicsItem::ItemSendsScenePositionChanges);
	m_dataProxy->setAcceptHoverEvents(true);

	m_dataProxy->setZValue(DATA_Z_VALUE);
}

void DiagramBox::setOutputSlotPos()
{
	QPointF p = (boundingRect().bottomRight() + boundingRect().topRight()) / 2;
	p.rx() -= 7; // Offset by 7px because the bounding rect of a box is offset to include the publish icon
	p.rx() += 5; // Set a bit of margin to the right to prevent the round-shape to overlap
	m_outputSlot->setPos(p);
}

/**
 * @brief DiagramBox::scriptName returns the name of the script this box is in, if it exists
 * @return
 */
QString DiagramBox::scriptName()
{
	// First get the scene
	DiagramScene *dScene = dynamic_cast<DiagramScene *>(scene());
	if (dScene == nullptr) {
		qWarning() << "[DiagramBox] not able to get or cast the DiagramScene";
		return "";
	}

	Script *script = dScene->script();
	if (script == nullptr) {
		qWarning() << "[DiagramBox] not able to fetch Script from DiagramScene";
		return "";
	}

	return script->name();
}
