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

#include "activityfetcher.h"
#include "helpers.h"

ActivityFetcher::ActivityFetcher(const QString &topicName, DiagramBox *box, QObject *parent)
    : QThread(parent),
      m_topicName(topicName),
      m_box(box),
      m_shouldQuit(false)
{
	// Make sure we have a box
	if (m_box == nullptr)
		informUserAndCrash(tr("ActivityFetcher created without a DiagramBox, crashing."));

	start();
}

void ActivityFetcher::run()
{
	// Wait for the ROS master to become online
	while (!ros::master::check()) {
		if (m_shouldQuit) {
			quit();
			return;
		}
		msleep(100); // We cannot use ROS rate now because we need the ROS master to come up before
	}

	// Here, the ROS master is online and we can talk to it
	ros::NodeHandle nh;
	ros::Subscriber subscriber;

	switch (m_box->outputType()) {
		case SCALAR:
			subscriber = nh.subscribe(m_topicName.toStdString(),
			                          1,
			                          &ActivityFetcher::fetchScalar,
			                          this);
		break;

		case MATRIX:
			subscriber = nh.subscribe(m_topicName.toStdString(),
			                          1,
			                          &ActivityFetcher::fetchMatrix,
			                          this);
		break;

		default:
			qDebug() << "ActivityFetcher only supports SCALAR and MATRIX output type";
		return;
		break;
	}

	ros::Rate rate(10); // 10Hz
	while (ros::ok()) {
		if (m_shouldQuit) {
			quit();
			return;
		}

		ros::spinOnce();
		rate.sleep();
	}
}

void ActivityFetcher::fetchScalar(const std_msgs::Float64::ConstPtr &scalar)
{
	emit newScalar(scalar->data);
}

void ActivityFetcher::fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr &mat)
{
	QVector<qreal> *matrix = new QVector<qreal>;
	int size = mat->data.size();

	for (int i = 0; i < size; i += 1)
		*matrix << mat->data.at(i);

	emit newMatrix(matrix);
}

bool ActivityFetcher::shouldQuit() const
{
	return m_shouldQuit;
}

void ActivityFetcher::setShouldQuit(bool shouldQuit)
{
	m_shouldQuit = shouldQuit;
}
