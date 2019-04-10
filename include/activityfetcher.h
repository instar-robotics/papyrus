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

#ifndef ACTIVITYFETCHER_H
#define ACTIVITYFETCHER_H

#include "diagrambox.h"

#include <QThread>

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

/**
 * @brief The ActivityFetcher class is a thread that subscribes to a box's topic in order to
 * fetch its activity output, and then push the values to a visualizer.
 */

class ActivityFetcher : public QThread
{
	Q_OBJECT
public:
	explicit ActivityFetcher(const QString &topicName, DiagramBox *box, QObject *parent = nullptr);
//	explicit ActivityFetcher(const QString &topicName, DiagramBox *box, ActivityVisualizer *vis, QObject *parent = nullptr);
	~ActivityFetcher();

	void run() override;

private:
	void fetchScalar(const std_msgs::Float64::ConstPtr& scalar);
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& mat);

	QString m_topicName;
	DiagramBox *m_box;
	bool m_shouldQuit;

signals:
	void newScalar(qreal scalar);
//	void newMatrix(QList<qreal> *matrix);
	void newMatrix(QVector<qreal> *mat);
};

#endif // ACTIVITYFETCHER_H
