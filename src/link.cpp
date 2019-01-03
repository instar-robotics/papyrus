#include "link.h"
#include "diagrambox.h"
#include "diagramscene.h"
#include "helpers.h"

#include <QPainter>
#include <QDebug>
#include <QGraphicsScene>
#include <QStyleOptionGraphicsItem>
#include <QPainterPath>
#include <QPainterPathStroker>

Link::Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent) : QGraphicsItem(parent),
                                    m_from(f),
                                    m_to(t),
                                    m_secondary(checkIfSelfLoop()), // valable for initialisation
                                    m_selfLoop(checkIfSelfLoop()),
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
}

QRectF Link::boundingRect() const
{
	// If the line is not self-looping, then returns the bounding rect of the main line
	if (!m_selfLoop) {
		return m_line.boundingRect();
	} else {
		// Otherwise, return the intersection of the three line's bounding rects
		return m_leftSegment.boundingRect().united(m_line.boundingRect()).united(m_rightSegment.boundingRect());
	}
}

void Link::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	QColor color(48, 140, 198);
	QPen pen(color);
	qreal width = pen.widthF();

	if (option->state & QStyle::State_MouseOver) {
		width += 1;
	}

	// If the link is selected, set the bold value (prevent changing it when hovering)
	if (option->state & QStyle::State_Selected) {
		width = 4;
	}

	// If the link is invalid, set the bold value and the color
	if (m_isInvalid) {
		width = 5;
		pen.setColor(Qt::red);
	}

	pen.setWidthF(width);

	// Create a copy of option and remove the State_Selected option to prevent displaying an ugly
	// dotted rectangle
	QStyleOptionGraphicsItem newOption(*option);
	newOption.state.setFlag(QStyle::State_Selected, false);

	if (m_selfLoop || m_secondary)
		pen.setStyle(Qt::DashLine);

	if (!m_selfLoop) {
		m_line.setPen(pen);

		m_line.paint(painter, &newOption, widget);

		// Paint the weight
//        if (!isStringLink() && (m_to->inputType() == SCALAR_SCALAR || m_to->inputType() == SCALAR_MATRIX)) {
		if (isStringLink() || m_to->inputType() == SCALAR_SCALAR || m_to->inputType() == SCALAR_MATRIX) {
			QPen currPen = painter->pen();

			// Paint the weight a different color when it's negative
			if (!isStringLink() && m_weight < 0) {
				painter->setPen(QColor(198, 65, 242));
			}

			// Compute normal vector to slightly translate the line's bounding rect so that the
			// weight is not written *on* the line (less readable), but slightly above it
			QLineF nV = m_line.line().normalVector();
			nV.translate(mapToScene((m_line.line().p2() - m_line.line().p1()) / 2));

			QRectF offsetRect = boundingRect().translated((nV.p2() - nV.p1()) / 20);

			if (!isStringLink()) {
				painter->drawText(offsetRect, Qt::AlignCenter | Qt::TextDontClip, QString::number(m_weight));

				// Restore the previous brush
				if (m_weight < 0)
					painter->setPen(currPen);
			} else {
				painter->drawText(offsetRect, Qt::AlignCenter | Qt::TextDontClip, m_value);
			}
		}
	} else {
		m_line.setPen(pen);
		m_leftSegment.setPen(pen);
		m_rightSegment.setPen(pen);

		m_leftSegment.paint(painter, &newOption, widget);
		m_line.paint(painter, &newOption, widget);
		m_rightSegment.paint(painter, &newOption, widget);

		QPen currPen = painter->pen();

		// Paint the weight a different color when it's negative
		if (!isStringLink() && m_weight < 0) {
			painter->setPen(QColor(198, 65, 242));
		}

		// Paint the weight
		if (isStringLink() || m_to->inputType() == SCALAR_SCALAR || m_to->inputType() == SCALAR_MATRIX) {
			QRectF r = m_line.boundingRect();
			r.setTop(r.top() - 30);

			if (!isStringLink())
				painter->drawText(r, Qt::AlignCenter, QString::number(m_weight));
			else
				painter->drawText(r, Qt::AlignCenter, m_value);
		}

		// Restore the previous brush
		if (!isStringLink() && m_weight < 0)
			painter->setPen(currPen);
	}
}

QPainterPath Link::shape() const
{
	if (!m_selfLoop) {
		QPainterPathStroker *stroke = new QPainterPathStroker();
		stroke->setWidth(60.0);
		return stroke->createStroke(m_line.shape());
	} else {
		// This is not perfect, for some reason the m_line path is not symmetric
		QPainterPath path(m_rightSegment.shape());
		path.addPath(m_line.shape());
		path.addPath(m_leftSegment.shape());

		QPainterPathStroker *stroke = new QPainterPathStroker();
		stroke->setWidth(20.0);
		stroke->setJoinStyle(Qt::RoundJoin);
		return stroke->createStroke(path);
	}
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

/**
 * @brief Link::addLinesToScene adds the segments that constitutes this Link to the scene.
 * Unfortunately we need to have a separate function: it cannot be done in the constructor, because
 * when we call new Link(from, to), the Link object is not yet added to the scene, so it cannot do
 * it at this moment.
 */
void Link::addLinesToScene()
{
	DiagramScene *dscene = dynamic_cast<DiagramScene *>(scene());
	if (dscene == NULL)
		informUserAndCrash("Could not cast scene in DiagramScene");

	if (!m_selfLoop) {
		dscene->addItem(&m_line);
	} else {
		dscene->addItem(&m_leftSegment);
		dscene->addItem(&m_line);
		dscene->addItem(&m_rightSegment);
	}

	updateLines();
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

bool Link::checkIfSelfLoop()
{
	// If either 'from' or 'to' is NULL, then it cannot be a secondary link
	if (m_from == NULL || m_to == NULL) {
		return false;
	}
	// then check if some slots are not associated
	else if (m_from->box() == NULL || m_to->box() == NULL) {
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

void Link::updateLines()
{
	// Don't do anything if either 'from' or 'to' is null
	if (m_from == NULL || m_to == NULL)
		return;

	QPointF orig = mapFromItem(m_from, m_from->boundingRect().center());
	QPointF end  = mapFromItem(m_to, m_to->boundingRect().center());

	// Create just a single line when not secondary link
	if (!m_selfLoop) {
		m_line.setLine(QLineF(orig, end));
	} else {
		// When in secondary, we have to create two vertical lines
		QPointF one = orig;
		QPointF two = end;

		// We need a way to differentiate multiple self-links on the same function box
		// this is why we use the yDiff to offset the values for the segments
		qreal yDiff = one.y() - two.y();

		one.ry() -= 75 + yDiff;
		two.setY(one.y());

		one.rx() -= qAbs(yDiff);
		two.rx() += qAbs(yDiff);

		m_leftSegment.setLine(QLineF(two, end));
		m_rightSegment.setLine(QLineF(orig, one));
		m_line.setLine(QLineF(one, two));
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

	// First check if the types match
	m_isInvalid = !canLink(m_from->outputType(), m_to->inputType());

	// Early return if false
	if (m_isInvalid) {
		m_invalidReason = TYPES_INCOMPATIBLE;
		updateTooltip();
		return false;
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
	return m_isInvalid;
}

