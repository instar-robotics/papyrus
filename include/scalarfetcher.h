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

#ifndef SCALARFETCHER_H
#define SCALARFETCHER_H

#include "datafetcher.h"
#include "types.h"

#include <QDebug>
#include <QBarSet>
#include <QSplineSeries>

#include <ros/ros.h>
#include "std_msgs/Float64.h"

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization;

class ScalarFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;

	void fetchScalar(const std_msgs::Float64::ConstPtr& scalar);

private:
	ScalarVisualization *m_scalarVisualization;
};

#endif // SCALARFETCHER_H
