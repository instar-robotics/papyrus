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

#ifndef SCALARVISUALIZATION_H
#define SCALARVISUALIZATION_H

#include "types.h"
#include "datavisualization.h"
#include "rossession.h"

#include <QBarSet>
#include <QBarSeries>
#include <QHorizontalBarSeries>
#include <QChart>
#include <QChartView>
#include <QSplineSeries>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QList>

#include <vector>

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization : public DataVisualization
{
	Q_OBJECT
public:
	ScalarVisualization(QWidget *parent = nullptr,
	                    ROSSession *rosSession = nullptr,
	                    QGraphicsScene *scene = nullptr,
	                    DiagramBox *box = nullptr);
	void mousePressEvent(QMouseEvent *evt);

	void updateBarValues(const std::vector<qreal> &values);
	void pushGraphValues(const std::vector<qreal> &values);

protected:
	void createCharts();

	int m_size;
	int m_idx;

	QBarSet *m_barSet;
	QBarSeries *m_barSeries;
	QHorizontalBarSeries *m_horizontalBarSeries;
	QChart *m_barChart;
	QValueAxis *m_barAxisY;
	QValueAxis *m_barAxisX;
	QChartView *m_barView;
	qreal m_barMin;
	qreal m_barMax;
	MatrixShape m_matrixShape;  // used to determine wether to display a ROW or COL vector (when applicable)

	QList<QSplineSeries *> m_graphSeries;
	QChart *m_graphChart;
	QVBoxLayout *m_vLayout;
	QValueAxis *m_graphAxisX;
	QValueAxis *m_graphAxisY;
	QChartView *m_graphView;
	qreal m_graphMin;
	qreal m_graphMax;

protected slots:
	void switchToBar();
	void switchToGraph();
};

#endif // SCALARVISUALIZATION_H
