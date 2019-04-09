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

#ifndef DIAGRAMCHART_H
#define DIAGRAMCHART_H

#include "diagrambox.h"
#include "types.h"

#include <QChart>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QBarSet>
#include <QValueAxis>
#include <QGraphicsSceneHoverEvent>

QT_CHARTS_USE_NAMESPACE

class DiagramChart : public QChart
{
	Q_OBJECT

public:
	explicit DiagramChart(DiagramBox *box,
	                      QGraphicsItem *parent = nullptr,
	                      Qt::WindowFlags wFlags = Qt::WindowFlags());

	void hoverMoveEvent(QGraphicsSceneHoverEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

private:
	DiagramBox *m_box;          // The associated DiagramBox
	int m_size;                 // The number of data points in this chart
	MatrixShape m_matrixShape;  // Used to know if we display horizontally or vertically

	QBarSet m_barSet;

	QBarSeries m_barSeries;
	QValueAxis m_yAxis;

	QHorizontalBarSeries m_horizontalBarSeries;
	QValueAxis m_xAxis;

	qreal m_barMin;
	qreal m_barMax;

	qreal m_width;
	qreal m_height;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing

public slots:
	void updateBarValues(QList<qreal> *matrix);
};

#endif // DIAGRAMCHART_H
