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

#ifndef MATRIXFETCHER_H
#define MATRIXFETCHER_H

#include "scalarvisualization.h"

#include <QString>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

class ScalarVisualization;
class MatrixVisualization;

class MatrixFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, VisualizationType type, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& mat);

private:
	ScalarVisualization *m_scalarVisualization;
	MatrixVisualization *m_matrixVisualization;
	QList<double> m_dataList;
};

#endif // MATRIXFETCHER_H
