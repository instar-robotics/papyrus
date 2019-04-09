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

#include "diagramchart.h"
#include "helpers.h"
#include "constants.h"

DiagramChart::DiagramChart(DiagramBox *box,
                           QGraphicsItem *parent,
                           Qt::WindowFlags wFlags)
    : QChart(parent, wFlags),
      m_box(box),
      m_barSet(""),
      m_barMin(-1),
      m_barMax(1),
      m_resizeType(NO_RESIZE)
{
	if (m_box == nullptr)
		informUserAndCrash(tr("DiagramChart created with null pointer in place of box!"));

	// Set the initial size and the shape of the chart
	qreal x, y;
	if (m_box->outputType() == SCALAR || (m_box->outputType() == MATRIX
	                                      && m_box->rows() == 1
	                                      && m_box->cols() == 1)) {
		m_size = 1;
		m_matrixShape = POINT;
		m_width = 150;
		m_height = 150;
	} else if (m_box->outputType() == MATRIX && m_box->rows() == 1) {
		m_size = m_box->cols();
		m_matrixShape = ROW_VECT;
		m_width = 350;
		m_height = 220;
	} else if (m_box->outputType() == MATRIX && m_box->cols() == 1) {
		m_size = m_box->rows();
		m_matrixShape = COL_VECT;
		m_width = 260;
		m_height = 350;
	} else {
		informUserAndCrash(tr("Cannot create charts: not a scalar nor a vector!"));
		return;
	}

	// Initialize values with 0
	for (int i = 0; i < m_size; i += 1)
		m_barSet << 0;

	// Bars are colored in a light blue tone
	m_barSet.setColor(QColor(51, 153, 255));

	QValueAxis *axis = nullptr;

	// Display bars horizontally for POINT and ROW_VECT shapes
	if (m_matrixShape == POINT || m_matrixShape == ROW_VECT) {
		m_barSeries.append(&m_barSet);
		m_barSeries.setLabelsVisible(false);

		addSeries(&m_barSeries);

		axis = &m_yAxis;

		addAxis(&m_yAxis, Qt::AlignLeft);
	}
	// Display bars vertically for COL_VECT shape
	else if (m_matrixShape == COL_VECT) {
		m_horizontalBarSeries.append(&m_barSet);
		m_horizontalBarSeries.setLabelsVisible(false);

		addSeries(&m_horizontalBarSeries);

		axis = &m_xAxis;

		addAxis(&m_xAxis, Qt::AlignBottom);
	} else {
		informUserAndCrash(tr("DiagramChart only supports POINT, ROW_VECT and COL_VECT shapes!"));
		return;
	}

	// Parameterize the axis
	axis->setRange(m_barMin, m_barMax);
	axis->setTickCount(9); // Keep it odd so that 0 is always displayed

	// Shrink down font size to prevent "..." being displayed
	QFont axisFont = axis->labelsFont();
	axisFont.setPointSize(axisFont.pointSize() - 6);
	axis->setLabelsFont(axisFont);

	// Chart's title set to the box's title if it exists, defaults to function name
	QString title = m_box->name();
	title = m_box->title().isEmpty() ? title : m_box->title();
	setTitle(title);

	setFlag(QGraphicsItem::ItemSendsGeometryChanges);
	setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
	setAcceptHoverEvents(true);

	// Set the background of the chart in a light gray tone
	setBackgroundBrush(QBrush(QColor(239, 239, 239)));

	setZValue(DATA_Z_VALUE);

	x = m_box->scenePos().x();
	y = m_box->scenePos().y() - m_height - 10; // Move the chart slightly above the box
	setGeometry(x, y, m_width, m_height);
	legend()->hide();

	// Disable animations (introduces unwanted value smoothing)
	setAnimationOptions(QChart::NoAnimation);

	// Remove margins because we want to maximise graph's visibility
	setMargins(QMargins(-0, 0, 0, 0));

	// Reduce title's font size
	QFont tFont = titleFont();
	tFont.setPointSize(tFont.pointSize() - 3);
	setTitleFont(tFont);
}

/**
 * @brief DiagramChart::hoverMoveEvent is used to change the cursor's shape based on its position
 * inside the graph, to hint that it can be resized.
 * @param evt
 */
void DiagramChart::hoverMoveEvent(QGraphicsSceneHoverEvent *evt)
{
	qreal mouseX = evt->pos().x();
	qreal mouseY = evt->pos().y();
	qreal margin = 20;

	if (mouseX >= m_width - margin && mouseY >= m_height - margin)
		setCursor(Qt::SizeFDiagCursor);
	else if (mouseX >= m_width - margin)
		setCursor(Qt::SizeHorCursor);
	else if (mouseY >= m_height - margin)
		setCursor(Qt::SizeVerCursor);
	else
		setCursor(Qt::ArrowCursor);

	QChart::hoverMoveEvent(evt);

}

/**
 * @brief DiagramChart::mousePressEvent is used to temporarily disable the graph's being movable
 * and switch the graph into the correct resizing move, based on where the use clicked its mouse.
 * @param evt
 */
void DiagramChart::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
	qreal mouseX = evt->pos().x();
	qreal mouseY = evt->pos().y();
	qreal margin = 20;

	if (mouseX >= m_width - margin && mouseY >= m_height - margin)
		m_resizeType = RESIZE_BOTTOM_RIGHT;
	else if (mouseX >= m_width - margin)
		m_resizeType = RESIZE_RIGHT;
	else if (mouseY >= m_height - margin)
		m_resizeType = RESIZE_BOTTOM;
	else
		m_resizeType = NO_RESIZE;

	if (m_resizeType == NO_RESIZE)
		setFlag(QGraphicsItem::ItemIsMovable, true);
	else
		setFlag(QGraphicsItem::ItemIsMovable, false);

	QChart::mousePressEvent(evt);
}

/**
 * @brief DiagramChart::mouseReleaseEvent is used to switch off resizing mode when releasing the
 * mouse button
 * @param evt
 */
void DiagramChart::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
	m_resizeType = NO_RESIZE;

	QChart::mouseReleaseEvent(evt);
}

/**
 * @brief DiagramChart::mouseMoveEvent is used to resize the chart when it is in resizing mode.
 * @param evt
 */
void DiagramChart::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
	// Define some minimum dimensions so we can't make charts too small
	qreal minWidth = 100;
	qreal minHeight = 100;
	qreal newWidth, newHeight;

	switch (m_resizeType) {
		case RESIZE_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= minWidth ? newWidth : minWidth;
		break;

		case RESIZE_BOTTOM:
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= minHeight ? newHeight : minHeight;
		break;

		case RESIZE_BOTTOM_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= minWidth ? newWidth : minWidth;
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= minHeight ? newHeight : minHeight;
		break;

		default:
			;
	}

	resize(m_width, m_height);

	QChart::mouseMoveEvent(evt);
}

// TODO: use pointers instead of copy?
void DiagramChart::updateBarValues(const QList<qreal> matrix&)
{
	qDebug() << "[DiagramChart] received" << matrix.size() << "values in a matrix";
}
