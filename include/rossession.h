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
#include "scopemessage.h"
#include "rttokenmessage.h"

#include <QThread>
#include <QString>
#include <QSet>

#include "diagnostic_msgs/KeyValue.h"
#include "hieroglyph/OscilloArray.h"
#include "hieroglyph/OscilloData.h"
#include "hieroglyph/RtToken.h"

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QThread
{
	Q_OBJECT

public:
	ROSSession(const QString &nodeName, QObject *parent = nullptr);
	~ROSSession();

	void addToHotList(QSet<QUuid> uuids);

	bool callServiceControl(QString cmd);
	ScriptStatus queryScriptStatus();

	bool callServiceOscillo(const QString &cmd);
	void registerOscillo();
	void registerRTToken();

	bool callServiceRTToken(const QString &cmd);
	bool callServiceComment(bool comment, QList<QUuid> uuids);
	bool callServiceWeight(const QString &cmd, const QString& filepath = "");

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

	void setShouldStartRTToken(bool shouldStartRTToken);

private:
	ros::NodeHandle *m_nh;   // The node handle used to interact with ROS
	bool m_shouldQuit;       // Used to properly exit the thread
	QString m_nodeName;      // Name of the node it should listen
	QSet<QUuid> m_hotList;   // List of functions' uuids whose output to activate on-the-fly
	bool m_isFirstRun;       // To differentiate a 'resume' on first launch or after a pause
	QList<ros::Subscriber> m_subs; // A list of subscribers not to loose their scope

	bool m_shouldStartRTToken; // Whether we are schedulded to start the RT Token next time we detect the script to start

	void run() override;
	void handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	void activateOutputs(QSet<QUuid> uuids);
	void handleOscilloMessage(const hieroglyph::OscilloArray::ConstPtr& msg);
	void handleRTTokenMessage(const hieroglyph::RtToken::ConstPtr& msg);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
	void newOscilloMessage(RTTokenMessage *rtTokenMessage, QVector<ScopeMessage> *scopeMessages);
	void newRTTokenMessage(RTTokenMessage *rtTokenMessage);
};

#endif // ROSSESSION_H
