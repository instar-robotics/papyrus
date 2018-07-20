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
                                    m_isInvalid(false)
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
    } else {
        m_line.setPen(pen);
        m_leftSegment.setPen(pen);
        m_rightSegment.setPen(pen);

        m_leftSegment.paint(painter, &newOption, widget);
        m_line.paint(painter, &newOption, widget);
        m_rightSegment.paint(painter, &newOption, widget);
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
 * @brief Link::checkIfInvalid checks that dimensions of the two boxes it is connected to are the
 * same when the type is SCALAR_MATRIX
 */
bool Link::checkIfInvalid()
{
    // Pointers check
    if (m_to == NULL)
        informUserAndCrash(tr("Can't check if Link is invalid: no InputSlot"));

    if (m_from == NULL)
        informUserAndCrash(tr("Can't check if Link is invalid: no OutputSlot"));

    if (m_to->inputType() == SCALAR_MATRIX) {
        DiagramBox *box = m_to->box();
        if (box == NULL)
            informUserAndCrash(tr("Can't check if Link is invalid: no destination box"));
        int toRows = box->rows();
        int toCols = box->cols();

        box = m_from->box();
        if (box == NULL)
            informUserAndCrash(tr("Can't check if Link is invalid: no origin box"));
        int fromRows = box->rows();
        int fromCols = box->cols();

        // I *know* parentheses are optional here, but they increase readability, so sue me
        m_isInvalid = (toRows != fromRows) || (toCols != fromCols);
    } else {
        // This is considered valid
        m_isInvalid = false;
    }

    // At the end, return wether we are invalid
    return m_isInvalid;
}

