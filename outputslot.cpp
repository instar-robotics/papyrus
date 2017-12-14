#include "outputslot.h"

#include <QPainter>

OutputSlot::OutputSlot(QString &name) : Slot(name)
{

}

std::set<Arrow *> OutputSlot::outputs() const
{
    return m_outputs;
}

void OutputSlot::addOutput(Arrow *output)
{
    if (output == NULL)
        return;

    m_outputs.insert(output);
}

void OutputSlot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->drawRect(QRectF(0, 0, 50, 50));
}
