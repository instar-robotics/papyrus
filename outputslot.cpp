#include "outputslot.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QDebug>

OutputSlot::OutputSlot() : Slot()
{
//    setFlags(QGraphicsItem::ItemIsSelectable
//           | QGraphicsItem::ItemIsMovable
//           | QGraphicsItem::ItemSendsScenePositionChanges);
//    setAcceptHoverEvents(true);
}

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
    Q_UNUSED(widget);

    /*
    if (option->state & QStyle::State_Selected) {
        qDebug() << "Output slot is selected";
    } else {
        qDebug() << "Output not selected" << i++;
    }
    //*/

    painter->drawEllipse(QPointF(0, 0), 5, 5);
}

QRectF OutputSlot::boundingRect() const
{
    return QRectF(-5, -5, 10, 10);
}
