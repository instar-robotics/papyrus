#include "link.h"
#include "diagrambox.h"
#include "diagramscene.h"

#include <QPainter>
#include <QDebug>
#include <QGraphicsScene>
#include <QStyleOptionGraphicsItem>

Link::Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent) : QGraphicsItem(parent),
                                    m_from(f),
                                    m_to(t),
                                    m_secondary(checkIfSecondary())
{
    m_uuid = QUuid::createUuid();

    // Add ourselves as input and output to the corresponding slots
    f->addOutput(this);
    t->addInput(this);
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
    if (!m_secondary) {
        m_line.paint(painter, option, widget);
    } else {
        m_leftSegment.paint(painter, option, widget);
        m_line.paint(painter, option, widget);
        m_rightSegment.paint(painter, option, widget);
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
        qFatal("Could not cast scene in DiagramScene");

    if (!m_secondary) {
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
    else {
        return false;
    }
}

void Link::updateLines()
{
    // Don't do anything if either 'from' or 'to' is null
    if (m_from == NULL || m_to == NULL)
        return;

    QPointF orig = mapFromItem(m_from, m_from->boundingRect().center());
    QPointF end  = mapFromItem(m_to, m_to->boundingRect().center());

    // Create just a single line when not secondary link
    if (!m_secondary) {
        m_line.setLine(QLineF(orig, end));
    } else {
        // When in secondary, we have to create two vertical lines
        QPointF one = orig;
        one.ry() -= 50;

        QPointF two = end;
        two.ry() -= 50;

        m_leftSegment.setLine(QLineF(two, end));
        m_rightSegment.setLine(QLineF(orig, one));
        m_line.setLine(QLineF(one, two));
    }
}

