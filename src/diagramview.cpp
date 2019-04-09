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

#include "diagramview.h"
#include "constants.h"
#include "librarypanel.h"

#include <iostream>
#include <QWheelEvent>
#include <QMimeData>
#include <QTreeWidgetItem>
#include <QDebug>

DiagramView::DiagramView(QWidget *parent) : QGraphicsView(parent)
{
	// Make it so that transformations (essentially zooming) are centered on mouse
	setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	setMouseTracking(true);
	setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	setDragMode(QGraphicsView::RubberBandDrag);
}

DiagramView::DiagramView(QGraphicsScene *scene, QWidget *parent) : DiagramView(parent)
{
	setScene(scene);
}

/**
 * @brief DiagramView::wheelEvent handles scrolling and zooming
 */
void DiagramView::wheelEvent(QWheelEvent *evt)
{
	// Zoom if CTRL is pressed while scrolling
	if (evt->modifiers() & Qt::ControlModifier) {

		// Handle direction of zoom
		if (evt->delta() > 0)
			scale(SCALE_FACTOR, SCALE_FACTOR);
		else
			scale(1 / SCALE_FACTOR, 1 / SCALE_FACTOR);
	} else {
		// If CTRL is not pressed, simply scroll
		QGraphicsView::wheelEvent(evt);
	}
}
