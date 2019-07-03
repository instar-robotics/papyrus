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

#include "link.h"
#include "diagrambox.h"
#include "diagramscene.h"
#include "helpers.h"
#include "constants.h"

#include <QPainter>
#include <QDebug>
#include <QGraphicsScene>
#include <QStyleOptionGraphicsItem>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QGraphicsTextItem>

Link::Link(OutputSlot *f, InputSlot *t, QGraphicsObject *parent) : QGraphicsObject(parent),
                                    m_from(f),
                                    m_to(t),
                                    m_secondary(checkIfSelfLoop()), // valable for initialisation
                                    m_selfLoop(checkIfSelfLoop()),
                                    m_line(this),
                                    m_leftSegment(this),
                                    m_rightSegment(this),
                                    m_weight(1.0),
                                    m_isInvalid(false),
                                    m_connectivity(ONE_TO_ONE) // chose one by default
{
	m_uuid = QUuid::createUuid();

	// Add ourselves as input and output to the corresponding slots
	if (f != NULL)
		f->addOutput(this);
	if (t != NULL)
		t->addInput(this);

	setAcceptHoverEvents(true);
	setFlag(QGraphicsItem::ItemIsSelectable, true);

	// If we are not self looping, hide the left and right segments
	if (!m_selfLoop) {
		m_rightSegment.hide();
		m_leftSegment.hide();
	}

	// Create the label used to display the weight or value
	m_label = new QGraphicsTextItem;

	// Shrink the font, which by default is quite big
	QFont labelFont = m_label->font();
	labelFont.setPointSizeF(labelFont.pointSizeF() - 4);
	m_label->setFont(labelFont);
	m_label->setDefaultTextColor(defaultLinkColor);

	setZValue(LINKS_Z_VALUE);
	m_label->setZValue(LINKS_Z_VALUE - 1);
}

Link::~Link()
{
	if (m_label != nullptr) {
		delete m_label;
		m_label = nullptr;
	}
}

QRectF Link::boundingRect() const
{
	// If the line is not self-looping, then returns the bounding rect of the main line
	if (!m_selfLoop) {
		// For some reason, when the line is perfectly horizontal, we need to be pixel-perfect to
		// select/over the lin, even though our shape() implementation produces a larger rectangle
		// I found that adjusting the boundingRect() this way (only when line is horizontal) solves
		// the issue. I'm not sure why, but it works so for now, this stays like this.
		// Elegant solution welcome.
		if (m_line.line().p1().y() == m_line.line().p2().y())
//			return m_line.boundingRect().adjusted(-10, -10, 10, 10);
			return m_line.boundingRect().adjusted(0, -10, 0, 10);
		else
			return m_line.boundingRect();
	} else {
		// Otherwise, return the intersection of the three line's bounding rects
		return m_leftSegment.boundingRect().united(m_line.boundingRect()).united(m_rightSegment.boundingRect());
	}
}

void Link::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(painter);
	Q_UNUSED(option);
	Q_UNUSED(widget);

	// Hide or show the left and right segments based on whether the link is self looping or not
	// TODO: maybe can be done once in constructor and that's all?
	if (m_selfLoop) {
		m_leftSegment.show();
		m_rightSegment.show();
	} else {
		m_leftSegment.hide();
		m_rightSegment.hide();
	}

	qreal width = defaultLinkWidth;
	QColor color = defaultLinkColor;

	// Increase the width and the boldness if the Link is selected
	if (isSelected()) {
		width += 1.3;
		setOpacity(1.0);
	}

	// If the link comes from or goes to a commented box, gray it out
	if (m_to->box()->isCommented() || m_from->box()->isCommented()) {
		color = QColor(Qt::gray).light();
	} else if (m_isInvalid) {
		// If the link is invalid, set the bold value and the color
		width = 3;
		color = Qt::red;
	}

	// Set final width and color
	m_leftSegment.setColor(color);
	m_line.setColor(color);
	m_rightSegment.setColor(color);
	m_label->setDefaultTextColor(color);

	m_line.setWidth(width);
	m_leftSegment.setWidth(width);
	m_rightSegment.setWidth(width);

	// Set the cursor to interogation when the link is invalid (to hint the user that there is some
	// message available).
	// NOTE: it will go back to arrow cursor thanks to hoverLeave event
	if (m_invalidReason != INVALID_INVALID_REASON) {
		setCursor(QCursor(Qt::WhatsThisCursor));
	}
}

QPainterPath Link::shape() const
{
	QPainterPathStroker *stroke = new QPainterPathStroker;
	stroke->setWidth(12.0);
	stroke->setJoinStyle(Qt::RoundJoin);

	if (!m_selfLoop) {
		return stroke->createStroke(m_line.shape());
	} else {
		QPainterPath path;
		path.addPath(stroke->createStroke(m_leftSegment.shape()));
		path.addPath(stroke->createStroke(m_line.shape()));
		path.addPath(stroke->createStroke(m_rightSegment.shape()));

		return path;
	}
}

QVariant Link::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
	if (change == QGraphicsItem::ItemSelectedHasChanged) {
		QFont labelFont = m_label->font();

		if (value.toBool())
			labelFont.setBold(true);
		else
			labelFont.setBold(false);

		m_label->setFont(labelFont);
	}

	return QGraphicsObject::itemChange(change, value);
}

/**
 * @brief Link::isStringLink tells whether this link is between a STRING and STRING_INPUT type.
 * This is a convenience function used to quicken the condition when one wants to access the link's
 * value
 * @return
 */
bool Link::isStringLink()
{
	if (m_from == NULL || m_to == NULL)
		informUserAndCrash(tr("Trying to check whether a Link is a string link, but it is missing "
		                      "either its from or to box!"));

	return m_from->outputType() == STRING && m_to->inputType() == STRING_INPUT;
}

/**
 * @brief Link::updateToolTip updates the tooltip of the Link based on the reason why it's invalid,
 * set it to empty string if it is valid
 */
void Link::updateTooltip()
{
	QString str;

	if (m_invalidReason & TYPES_INCOMPATIBLE)
		str += tr("<li>incompatible types between origin box and this slot's expected type</li>");

	if (m_invalidReason & SIZES_DONT_MATCH)
		str += tr("<li>size don't match for SCALAR_MATRIX type (checkSize = true)</li>");

	if (m_invalidReason & SHAPE_MUST_BE_POINT)
		str += tr("<li>slot is expecting a (1,1) matrix\n");
	else if (m_invalidReason & SHAPE_MUST_BE_VECT)
		str += tr("<li>slot is expecting a vector matrix (either a (N, 1) or a (1, N) matrix)</li>");
	else if (m_invalidReason & SHAPE_MUST_BE_ROW_VECT)
		str += tr("<li>slot is expecting a row vector (a (1, N) matrix)</li>");
	else if (m_invalidReason & SHAPE_MUST_BE_COL_VECT)
		str += tr("<li>slot is expecting a column vector (a (N, 1) matrix)</li>");

	if (!str.isEmpty())
		str = tr("Link is <strong>invalid</strong>:<ul>") + str + "</ul>";

	setToolTip(str);
}

QUuid Link::uuid() const
{
	return m_uuid;
}

void Link::setUuid(const QUuid &uuid)
{
	m_uuid = uuid;
}

OutputSlot *Link::from() const
{
	return m_from;
}

void Link::setFrom(OutputSlot *from)
{
	m_from = from;
	m_selfLoop = checkIfSelfLoop();
	updateLines();
}

InputSlot *Link::to() const
{
	return m_to;
}

void Link::setTo(InputSlot *to)
{
	m_to = to;
	m_selfLoop = checkIfSelfLoop();
	updateLines();
}

bool Link::secondary() const
{
	return m_secondary;
}

void Link::setSecondary(bool secondary)
{
	m_secondary = secondary;
}

/**
 * @brief Link::checkIfSelfLoop checks if the links comes from and goes to the same @DiagramBox
 * @return
 */
bool Link::checkIfSelfLoop()
{
	// If either 'from' or 'to' is NULL, then it cannot be a secondary link
	if (m_from == nullptr || m_to == nullptr) {
		return false;
	}
	// then check if some slots are not associated
	else if (m_from->box() == nullptr || m_to->box() == nullptr) {
		return false;
	}
	// otherwise, check if the associated boxes for 'from' and 'to' are the same
	else if (m_from->box() == m_to->box()) {
		return true;
	}
	else {
		return false;
	}
}

//*
QGraphicsTextItem *Link::label() const
{
	return m_label;
}

void Link::setLabel(QGraphicsTextItem *label)
{
	m_label = label;
}
//*/
QString Link::regexes() const
{
	return m_regexes;
}

void Link::setRegexes(const QString &regexes)
{
	m_regexes = regexes;
}

LinkInvalidReason Link::invalidReason() const
{
	return m_invalidReason;
}

void Link::setInvalidReason(const LinkInvalidReason &invalidReason)
{
	m_invalidReason = invalidReason;
}

Connectivity Link::connectivity() const
{
	return m_connectivity;
}

void Link::setConnectivity(const Connectivity &connectivity)
{
	m_connectivity = connectivity;
}

QString Link::value() const
{
	return m_value;
}

void Link::setValue(const QString &value)
{
	m_value = value;
}

bool Link::selfLoop() const
{
	return m_selfLoop;
}

bool Link::isInvalid() const
{
	return m_isInvalid;
}

void Link::setIsInvalid(bool isInvalid)
{
	m_isInvalid = isInvalid;
}

qreal Link::weight() const
{
	return m_weight;
}

void Link::setWeight(const qreal &weight)
{
	m_weight = weight;
}

/**
 * @brief Link::updateLines update the position of the QGraphicsLineItem so they "follow" the slots
 * to which they are linked.
 */
void Link::updateLines()
{
	// Don't do anything if either 'from' or 'to' is null
	if (m_from == NULL || m_to == NULL)
		return;

	QPointF orig = mapFromItem(m_from, m_from->boundingRect().center());
	QPointF end  = mapFromItem(m_to, m_to->boundingRect().center());

	// Paint the weight if not a string link
	//*
	if (!isStringLink()) {
		m_label->setHtml(QString("<center>%1</center>").arg(m_weight));
	} else {
		m_label->setHtml(QString("<center>%1</center>").arg(m_value));
	}
	//*/


	// Update the single line when it's not a self looping link
	if (!m_selfLoop) {
		// We need to make the Link's line connecting the slots slightly shorter by 2 x slot's radius
		// (half a radius at each extremity). This is so that links don't go above the slots. It
		// could not be solved by z-index, because slots's parent is the box, and the box's z-index
		// need to be less than the link's.
		QLineF line(orig, end);
		qreal length = line.length();
		qreal radiusOrig = m_from->radius();
		qreal radiusEnd = m_to->radius();

		QPointF newEnd = line.pointAt(1.0 - (radiusEnd+1) / length);
		QPointF newOrig = line.pointAt((radiusOrig+1) / length);
		line.setP1(newOrig);
		line.setP2(newEnd);
		m_line.setLine(line);
		m_label->setTextWidth(m_line.line().length());
		if (orig.x() <= end.x()) {
			QPointF p = m_line.scenePos();
			QLineF nV = m_line.line().normalVector();
			nV.setLength(20);
			m_label->setPos(p + nV.p2());
		} else {
			m_label->setPos(m_line.line().p2());
		}

		qreal oppositeSide = orig.y() - end.y();
		qreal adjacentSide = end.x() - orig.x();
		qreal rotationAmount = atan(oppositeSide / adjacentSide);
		m_label->setRotation(-rotationAmount / (2 * M_PI) * 360);
	} else {
		// When the link is self-looping, we have to create two vertical lines
		QPointF one = orig;
		QPointF two = end;

		// We need a way to differentiate multiple self-links on the same function box
		// this is why we use the yDiff to offset the values for the segments
		qreal yDiff = one.y() - two.y();

		one.ry() -= 75 + yDiff;
		two.setY(one.y());

		one.rx() -= qAbs(yDiff);
		two.rx() += qAbs(yDiff);

		// To prevent link overlapping with slot, see above
		orig.ry() -= m_from->radius() + 1;
		end.ry() -= m_to->radius() + 1;

		m_leftSegment.setLine(QLineF(two, end));
		m_rightSegment.setLine(QLineF(orig, one));
		m_line.setLine(QLineF(one, two));

		m_label->setTextWidth(m_line.line().length());
		QPointF endPos = two;
		endPos.ry() -= 20;
		m_label->setPos(endPos);
	}

	if (m_selfLoop || m_secondary) {
		m_leftSegment.setIsSecondary(true);
		m_line.setIsSecondary(true);
		m_rightSegment.setIsSecondary(true);
	} else {
		m_leftSegment.setIsSecondary(false);
		m_line.setIsSecondary(false);
		m_rightSegment.setIsSecondary(false);
	}
}

/**
 * @brief Link::checkIfInvalid checks if a given link is invalid. It can be invalid for several
 * reasons:
 * - when the ouput type of the origin box and the input type of the destination box are not
 *   compatible (i.e. cannot be linked)
 * - when the type is SCALAR_MATRIX and the 'checkSize' flag is true, dimensions must be the same,
 *   otherwise, that link is invalid
 * - when the destination inputSlot is a matrix type and has a requirement for shape, then the sizes
 *   of the incoming @DiagramBox must respect that shape requirement
 */
bool Link::checkIfInvalid()
{
	// Pointers check
	if (m_to == NULL)
		informUserAndCrash(tr("Can't check if Link is invalid: no InputSlot"));

	if (m_from == NULL)
		informUserAndCrash(tr("Can't check if Link is invalid: no OutputSlot"));

	DiagramBox *boxTo = m_to->box();
	if (boxTo == NULL) {
		informUserAndCrash(tr("Link with UUID %1 cannot be checked for invalidity: it has no "
		                      "destination box").arg(m_uuid.toString()));
	}

	DiagramBox *boxFrom = m_from->box();
	if (boxFrom == NULL) {
		informUserAndCrash(tr("Link with UUID %1 cannot be checked for invalidity: it has no "
		                      "origin box").arg(m_uuid.toString()));
	}

	int toRows = boxTo->rows();
	int toCols = boxTo->cols();

	int fromRows = boxFrom->rows();
	int fromCols = boxFrom->cols();

	// Start with a valid link
	m_isInvalid = false;
	m_invalidReason = INVALID_INVALID_REASON;
	m_label->setDefaultTextColor(defaultLinkColor);

	// First check if the types match
	m_isInvalid = !canLink(m_from->outputType(), m_to->inputType());

	// Early return if true
	if (m_isInvalid) {
		m_invalidReason = TYPES_INCOMPATIBLE;
		updateTooltip();
		m_label->setDefaultTextColor(Qt::red);
		return true; // why was it false here for so long?
	}

	// Check sizes only if:
	// - the target box's input slot is SCALAR_MATRIX
	// - the 'checkSize' flag is true (by default it is)
	// - the target box is also a matrix
	if (m_to->inputType() == SCALAR_MATRIX
	    && m_to->checkSize()
	    && boxTo->outputType() == MATRIX) {

		// I *know* parentheses are optional here, but they increase readability, so sue me
		bool checkSizeError = (toRows != fromRows) || (toCols != fromCols);
		m_isInvalid = m_isInvalid || checkSizeError;

		// Update reason if invalid
		if (checkSizeError)
			m_invalidReason = m_invalidReason | SIZES_DONT_MATCH;
	}

	// Check matrix shape requirement if target slot is matrix
	if ((m_to->inputType() == SCALAR_MATRIX || m_to->inputType() == MATRIX_MATRIX)) {
		LinkInvalidReason reason;
		bool shapesMismatch = !shapesMatch(boxFrom, m_to, &reason);
		m_isInvalid = m_isInvalid || shapesMismatch;

		// Update reason if invalid
		if (shapesMismatch)
			m_invalidReason = m_invalidReason | reason;
	}

	// At the end, return wether we are invalid
	updateTooltip();
	if (m_isInvalid)
		m_label->setDefaultTextColor(Qt::red);
	return m_isInvalid;
}

/**
 * @brief Link::hoverEnterEvent turns the cursor into a hand cursor, hinting the user that he can
 * interact with the Link. Also lower its opacity to further improve visual sight that this Link is
 * hovered on (useful when lots of Links are close together)
 * @param evt
 */
void Link::hoverEnterEvent(QGraphicsSceneHoverEvent *evt)
{
	setCursor(QCursor(Qt::PointingHandCursor));
	setOpacity(0.3);

	QGraphicsObject::hoverEnterEvent(evt);
}

/**
 * @brief Link::hoverLeaveEvent restore the normal cursor and opacity for this Link when we hover out
 * @param evt
 */
void Link::hoverLeaveEvent(QGraphicsSceneHoverEvent *evt)
{
	setCursor(QCursor(Qt::ArrowCursor));
	setOpacity(1.0);

	QGraphicsObject::hoverEnterEvent(evt);
}
