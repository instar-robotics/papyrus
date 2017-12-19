#include "inputslot.h"

#include <QPainter>
#include <QDebug>

InputSlot::InputSlot() : Slot()
{

}

InputSlot::InputSlot(QString &name) : Slot(name), m_allowMultiple(false)
{

}

bool InputSlot::allowMultiple() const
{
    return m_allowMultiple;
}

void InputSlot::setAllowMultiple(bool allowMultiple)
{
    m_allowMultiple = allowMultiple;
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
    if (!m_allowMultiple && !m_inputs.empty()) {
        emit slotFull();
        return;
    }

    m_inputs.insert(input);
}

void InputSlot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->drawEllipse(QPointF(0, 0), 5, 5);
}

QRectF InputSlot::boundingRect() const
{
    return QRectF(-5, -5, 10, 10);
}

