/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER

  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.

  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

#include "outputslot.h"
#include "helpers.h"

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
	qreal m_radius = 5;

	if (option->state & QStyle::State_MouseOver) {
		width += 1;
	}

	QGraphicsScene *scene_ = scene();
	DiagramScene *dscene = dynamic_cast<DiagramScene *>(scene_);

	if (dscene == nullptr) {
		qFatal("Could not cast the scene into a DiagramScene!");
	}

	QGraphicsLineItem *drawnLine = dscene->line();

	// Change the output slot's size only if not drawing a line and box is not commented
	if (drawnLine == nullptr && !m_box->isCommented()) {
		// Make the slot bigger when the mouse is near it
		qreal sizeOffset = (400 - m_dist) / 100; // Grows linearly with distance -> quadratic should be better

		m_radius += pow(sizeOffset, 2) / 6;
	}

	// Subtract the half the line's width to prevent drawing outside the boudingRect
	m_radius -= width / 2.0;

	pen.setWidth(width);
	painter->setPen(pen);

	painter->drawEllipse(QPointF(cx, cy), m_radius, m_radius);

	// Fill the output with the color associated to its type (use a QPainterPath because there is no
	// painter->fillEllipse()) (if it's not drawing a line)
	if (drawnLine == nullptr) {
		QPainterPath path;
		path.addEllipse(QPointF(cx, cy), m_radius, m_radius);
		painter->fillPath(path, getTypeColor(m_outputType));
	}
}

QRectF OutputSlot::boundingRect() const
{
	// Set to be the same size as the input slot, for arbitrary reason
	return QRectF(-7.7, -7.7, 15.4, 15.4);
}

/**
 * @brief OutputSlot::mousePressEvent disable the RubberBang drag mode for the scene during the
 * creation of the link and prevents propagating the click so that we can create a link even when
 * the box is inside a comment Zone
 * @param evt
 */
void OutputSlot::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
	Q_UNUSED(evt);

	m_isDrawingLine = true;

	// Do NOT propagate the event: this prevents items undereneath to receive the event
	//	QGraphicsItem::mousePressEvent(evt);
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
//		link->update();
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
