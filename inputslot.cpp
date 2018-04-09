#include "inputslot.h"

#include <cmath>

#include <QPainter>
#include <QDebug>
#include <QStyle>
#include <QStyleOptionGraphicsItem>
#include <diagramscene.h>

InputSlot::InputSlot() : Slot(),
                         m_multiple(false),
                         m_inputType(MATRIX_MATRIX),
                         m_canLink(false)
{

}

InputSlot::InputSlot(QString &name) : InputSlot()
{
    setName(name);
}

bool InputSlot::multiple() const
{
    return m_multiple;
}

void InputSlot::setMultiple(bool allowMultiple)
{
    m_multiple = allowMultiple;
}

std::set<Arrow *> InputSlot::inputs() const
{
    return m_inputs;
}

/**
 * @brief Add a new input Arrow to this slot
 * @param input: the new input to add
 */
void InputSlot::addInput(Arrow *input)
{
    if (input == NULL)
        return;

    // If the input slot is set not to allow multiple values, only add if the set is empty
    if (!m_multiple && !m_inputs.empty()) {
        emit slotFull();
        return;
    }

    m_inputs.insert(input);
}

/**
 * @brief InputSlot::paint the input slots are drawn bigger when the mouse nears them, and it is
 * currently drawing a link
 * @param painter
 * @param option
 * @param widget
 */
void InputSlot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);

    QPen pen;
    qreal width = 1.5;
    QColor color = Qt::black;
//    QFont font = painter->font();

    qreal cx = 0;
    qreal cy = 0;
    qreal rx = 5;
    qreal ry = 5;

    if (option->state & QStyle::State_MouseOver) {
        width += 1;
    }

    QGraphicsScene *scene_ = scene();
    DiagramScene *dscene = dynamic_cast<DiagramScene *>(scene_);

    if (dscene == NULL) {
        qFatal("Could not cast the scene into a DiagramScene!");
    }

    // Change the output slot's size only if drawing a line
    if (dscene->line() != NULL) {
        // Update color and size according to validity of current link (is there is one being created)
        if (m_canLink) {
            // Make the slot bigger when the mouse is near it
            qreal sizeOffset = (400 - m_dist) / 100; // Grows linearly with distance -> quadratic should be better

            rx += pow(sizeOffset, 2) / 6;
            ry += pow(sizeOffset, 2) / 6;


            color = Qt::green;
            width += 1;
        }
        else {
            color = Qt::lightGray;
        }
    }

    pen.setColor(color);

    // Subtract the half the line's width to prevent drawing outside the boudingRect
    rx -= width / 2.0;
    ry -= width / 2.0;
    pen.setWidth(width);

    painter->setPen(pen);

    painter->drawEllipse(QPointF(cx, cy), rx, ry);
}

QRectF InputSlot::boundingRect() const
{
    // This is the maximum size the circle can grow when mouse nears it
    return QRectF(-7.7, -7.7, 15.4, 15.4);
}

/**
 * @brief InputSlot::updateArrows updates this input slot's connected Arrows's ending point,
 * so that the 'paint' function will draw the line at from the right position
 */
void InputSlot::updateArrows()
{
    QPointF p1, p2;
    QLineF line;

    foreach (Arrow *arrow, m_inputs) {
        line = arrow->line();
        p1 = line.p1(); // origin stays the same because we're moving the destination
        p2 = scenePos(); // new destination of Arrow is this slot's position

        // Set the new line for this Arrow
        arrow->setLine(QLineF(p1, p2));
    }
}

InputType InputSlot::inputType() const
{
    return m_inputType;
}

void InputSlot::setInputType(const InputType &inputType)
{
    m_inputType = inputType;
}

bool InputSlot::canLink() const
{
    return m_canLink;
}

void InputSlot::setCanLink(bool canLink)
{
    m_canLink = canLink;
}

