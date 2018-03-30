#include "link.h"
#include "diagrambox.h"

#include <QPainter>

Link::Link(QGraphicsItem *parent) : QGraphicsItem(parent),
                                    m_from(NULL),
                                    m_to(NULL),
                                    m_secondary(false)
{
    m_uuid = QUuid::createUuid();
}

Link::~Link()
{
    delete m_from;
    delete m_to;
}

QRectF Link::boundingRect() const
{
    // If the line is not self-looping, then returns the bounding rect of the main line
    if (!m_secondary) {
        return m_line.boundingRect();
    } else {
        // Otherwise, return the intersection of the three line's bounding rects
        return m_leftSegment.boundingRect().united(m_line.boundingRect()).united(m_rightSegment.boundingRect());
    }
}

void Link::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->drawLine(m_line.line());

    if (m_secondary) {
        painter->drawLine(m_leftSegment.line());
        painter->drawLine(m_rightSegment.line());
    }
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
    m_secondary = checkIfSecondary();
    updateLines();
}

InputSlot *Link::to() const
{
    return m_to;
}

void Link::setTo(InputSlot *to)
{
    m_to = to;
    m_secondary = checkIfSecondary();
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

bool Link::checkIfSecondary()
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

    qFatal("Problem when checking if secondary");
}

void Link::updateLines()
{
    // Don't do anything if either 'from' or 'to' is null
    if (m_from == NULL || m_to == NULL)
        return;

    QPointF orig = m_from->boundingRect().center();
    QPointF end  = m_to->boundingRect().center();

    // Create just a single line when not secondary link
    if (!m_secondary) {
        m_line.setLine(QLineF(orig, end));
    } else {
        // When in secondary, we have to create two vertical lines
        QPointF one = orig;
        one.ry() += 50;

        QPointF two = end;
        two.ry() += 50;

        m_leftSegment.setLine(QLineF(two, end));
        m_rightSegment.setLine(QLineF(orig, one));
        m_line.setLine(QLineF(one, two));
    }
}

