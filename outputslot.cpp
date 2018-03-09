#include "outputslot.h"

#include <cmath>
#include <iostream>

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QDebug>

OutputSlot::OutputSlot() : Slot(), m_isDrawingLine(false)
{

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

    QPen pen;
    qreal width = 1.5;
//    QFont font = painter->font();

    qreal cx = 0;
    qreal cy = 0;
    qreal rx = 5;
    qreal ry = 5;

    if (option->state & QStyle::State_MouseOver) {
        width += 1;
    }

    // Make the slot bigger when the mouse is near it
    qreal sizeOffset = (400 - m_dist) / 100; // Grows linearly with distance
    rx += pow(sizeOffset, 2) / 6;
    ry += pow(sizeOffset, 2) / 6;

    pen.setWidth(width);
    painter->setPen(pen);

    painter->drawEllipse(QPointF(cx, cy), rx, ry);
}

QRectF OutputSlot::boundingRect() const
{
    return QRectF(-5, -5, 10, 10);
}

void OutputSlot::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{

    m_isDrawingLine = true;

    QGraphicsItem::mousePressEvent(evt);
}

bool OutputSlot::isDrawingLine() const
{
    return m_isDrawingLine;
}

void OutputSlot::setIsDrawingLine(bool isDrawingLine)
{
    m_isDrawingLine = isDrawingLine;
}
