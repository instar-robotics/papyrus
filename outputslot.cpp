#include "outputslot.h"

#include <cmath>
#include <iostream>

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QDebug>
#include <QGraphicsView>
#include <diagramscene.h>
#include <diagramview.h>

#include "link.h"

OutputSlot::OutputSlot() : Slot(),
                           m_isDrawingLine(false),
                           m_outputType(MATRIX)
{

}

OutputSlot::OutputSlot(QString &name) : OutputSlot()
{
    setName(name);
}

std::set<Link *> OutputSlot::outputs() const
{
    return m_outputs;
}

void OutputSlot::addOutput(Link *output)
{
    if (output == NULL)
        return;

    m_outputs.insert(output);
}

void OutputSlot::removeOutput(Link *output)
{
    if (output == NULL)
        return;

    m_outputs.erase(output);
}

/**
 * @brief OutputSlot::paint the output slots are drawn bigger as the cursor nears them, provided
 * the user is not in the process of creating a link
 * @param painter
 * @param option
 * @param widget
 */
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

    QGraphicsScene *scene_ = scene();
    DiagramScene *dscene = dynamic_cast<DiagramScene *>(scene_);

    if (dscene == NULL) {
        qFatal("Could not cast the scene into a DiagramScene!");
    }

    // Change the output slot's size only if not drawing a line
    if (dscene->line() == NULL) {
        // Make the slot bigger when the mouse is near it
        qreal sizeOffset = (400 - m_dist) / 100; // Grows linearly with distance -> quadratic should be better

        rx += pow(sizeOffset, 2) / 6;
        ry += pow(sizeOffset, 2) / 6;
    }

    // Subtract the half the line's width to prevetn drawing outside the boudingRect
    rx -= width / 2.0;
    ry -= width / 2.0;

    pen.setWidth(width);
    painter->setPen(pen);

    painter->drawEllipse(QPointF(cx, cy), rx, ry);
}

QRectF OutputSlot::boundingRect() const
{
    // Set to be the same size as the input slot, for arbitrary reason
    return QRectF(-7.7, -7.7, 15.4, 15.4);
}

/**
 * @brief OutputSlot::mousePressEvent disable the RubberBang drag mode for the scene during the
 * creation of the link
 * @param evt
 */
void OutputSlot::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    m_isDrawingLine = true;

    QGraphicsItem::mousePressEvent(evt);
}

/**
 * @brief OutputSlot::hoverEnterEvent deactivates the rubber band selection of the view(s) when
 * the cursor hovers an OuputSlot (so that when clicking to create a link doesn not create a
 * selection box)
 * @param evt
 */
void OutputSlot::hoverEnterEvent(QGraphicsSceneHoverEvent *evt)
{
    QList<QGraphicsView *> views = scene()->views();

    foreach (QGraphicsView *view, views) {
        // Change the view's drag mode to NoDrag only if it's a DiagramView (to prevent messing with
        // other views, like the minimap, etc.)
        DiagramView *view_ = dynamic_cast<DiagramView *>(view);
        if (view_ != NULL) {
            view_->setDragMode(QGraphicsView::NoDrag);
        }
    }

    Slot::hoverEnterEvent(evt);
}

/**
 * @brief OutputSlot::hoverLeaveEvent re-activate the rubber band selection of the views when the
 * cursor hovers out of an output slot, to restore the rubber band selection mode
 * @param evt
 */
void OutputSlot::hoverLeaveEvent(QGraphicsSceneHoverEvent *evt)
{
    QList<QGraphicsView *> views = scene()->views();

    foreach (QGraphicsView *view, views) {
        // Change the view's drag mode to rubber band only if it's a DiagramView (to prevent messing
        // with other views, like the minimap, etc.)
        DiagramView *view_ = dynamic_cast<DiagramView *>(view);
        if (view_ != NULL) {
            view_->setDragMode(QGraphicsView::RubberBandDrag);
        }
    }

    Slot::hoverLeaveEvent(evt);
}

bool OutputSlot::isDrawingLine() const
{
    return m_isDrawingLine;
}

void OutputSlot::setIsDrawingLine(bool isDrawingLine)
{
    m_isDrawingLine = isDrawingLine;
}

/**
 * @brief OutputSlot::updateLinks updates this output slot's connected Links's starting point,
 * so that the 'paint' function will draw the line from the right position
 */
void OutputSlot::updateLinks()
{
    foreach(Link *link, m_outputs) {
        link->updateLines();
        link->update();
    }
}

OutputType OutputSlot::outputType() const
{
    return m_outputType;
}

void OutputSlot::setOutputType(const OutputType &outputType)
{
    m_outputType = outputType;
}
