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

#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

#include "types.h"

#include <QThread>
#include <QString>
#include <QSet>

#include "diagnostic_msgs/KeyValue.h"

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QThread
{
	Q_OBJECT

public:
	ROSSession(const QString &nodeName, QObject *parent = nullptr);

	void addToHotList(QUuid uuid);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

private:
	bool m_shouldQuit;       // Used to properly exit the thread
	QString m_nodeName;      // Name of the node it should listen
	QSet<QUuid> m_hotList;   // List of functions' uuids whose output to activate on-the-fly
	bool m_isFirstRun;       // To differentiate a 'resume' on first launch or after a pause

	void run() override;
	void handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	void activateOutput(QUuid uuid);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
};

#endif // ROSSESSION_H
