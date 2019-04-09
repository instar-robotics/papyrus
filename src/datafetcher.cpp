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

#include "datafetcher.h"

#include <QDebug>

DataFetcher::DataFetcher(const QString &topicName, QObject *parent) :
    QThread(parent),
    m_topicName(topicName),
    m_shouldQuit(false)
{
//	m_sub = m_n.subscribe(topicName.toStdString(), 1000, &DataFetcher::fetchScalar, this);
}

// TODO: define a loop rate instead of spinning like hell
/*
void DataFetcher::run()
{
	qDebug() << "[DataFetcher] Run";
//	ros::Subscriber m_sub = m_n.subscribe(m_topicName.toStdString(), 1000, &DataFetcher::fetchScalar, this);

	ros::Rate rate(10); // 10 Hz
	while (ros::ok()) {
		if (m_shouldQuit) {
			qDebug() << "[DataFetcher] should quit now";
			quit();
			return;
		}

		ros::spinOnce();
		rate.sleep();
	}
}
//*/

/*
void DataFetcher::fetchScalar(const std_msgs::Float64::ConstPtr &scalar)
{
//	qDebug() << "[FetchScalar] called with value:" << scalar->data;
	qDebug() << "Replacing";
	m_barSet->replace(0, scalar->data);
	qDebug() << "Done replacing";
}
//*/

bool DataFetcher::shouldQuit() const
{
	return m_shouldQuit;
}

void DataFetcher::setShouldQuit(bool shouldQuit)
{
	m_shouldQuit = shouldQuit;
}

VisualizationType DataFetcher::visType() const
{
	return m_visType;
}
