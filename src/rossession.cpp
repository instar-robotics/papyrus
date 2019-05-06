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

#include "rossession.h"
#include "helpers.h"
#include "papyruswindow.h"
#include "hieroglyph/ArgCmd.h"

ROSSession::ROSSession(const QString &nodeName, QObject *parent)
    : QThread(parent),
      m_shouldQuit(false),
      m_nodeName(nodeName),
      m_isFirstRun(true)
{
	start();
}

ROSSession::~ROSSession()
{

}

/**
 * @brief ROSSession::addToHotList add a function to the list of functions for which we want to
 * activate the output when the script is run. Basically it waits for the script to be started, and
 * call @activateOutput() on each items.
 * If called when the script is already running, then also call @activateOutput() directly
 * @param uuid
 */
void ROSSession::addToHotList(QUuid uuid)
{
	if (!uuid.isNull()) {
		m_hotList.insert(uuid);

		// If the script has already been launched, then activate output immediately
		if (!m_isFirstRun)
			activateOutput(uuid);
	}
}

/**
 * @brief ROSSession::activateOutput issue a ROS service call to activate a function's output (i.e.
 * make this function publish its output on the bus)
 * IMPORTANT: this requires the script to be running. This is meant to be used internally. Users
 * should
 * @param uuid the UUID of the function which output we want to activate
 * @return whether it succeeded or not
 */
void ROSSession::activateOutput(QUuid uuid)
{
	QString srvName = m_nodeName + "/output";
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<hieroglyph::ArgCmd>(srvName.toStdString());
	hieroglyph::ArgCmd srv;
	srv.request.cmd = "start";
	srv.request.arg = uuid.toString().toStdString();

	if (!client.call(srv)) {
		qWarning() << "Failed to activate output on uuid" << uuid.toString() << "on node name" << m_nodeName;
	}
}

bool ROSSession::shouldQuit() const
{
	return m_shouldQuit;
}

void ROSSession::setShouldQuit(bool shouldQuit)
{
	m_shouldQuit = shouldQuit;
}

QString ROSSession::nodeName() const
{
	return m_nodeName;
}

void ROSSession::setNodeName(const QString &nodeName)
{
	m_nodeName = nodeName;
}

void ROSSession::run()
{
	// Wait for the ROS master to become online
	while (!ros::master::check()) {
		if (m_shouldQuit) {
			quit();
			return;
		}
		msleep(100); // We cannot use ROS rate now because we need the ROS master to come up before
	}

	ros::NodeHandle nh;
	ros::Rate rate(10); // 10Hz

	// Subscribe to the 'status' topic to listen to status change
	ros::Subscriber sub = nh.subscribe(QString("%1/%2").arg(m_nodeName, "status").toStdString(),
	                                   1000,
	                                   &ROSSession::handleStatusChange,
	                                   this);

	while (ros::ok()) {
		if (m_shouldQuit) {
			quit();
			return;
		}

		ros::spinOnce();
		rate.sleep();
	}
}

/**
 * @brief ROSSession::handleStatusChange handles messages from the ROS topic "/status" which is used
 * to indicate status changed (like paused, resumed, quit, etc.)
 * @param msg
 */
void ROSSession::handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr &msg)
{
	QString key = QString::fromStdString(msg->key).toLower();
	QString value = QString::fromStdString(msg->value).toLower();

	if (key == "control") {
		if (value == "resume") {
			// When the script is first run, activate all functions if the hot list
			if (m_isFirstRun) {
				foreach (QUuid uuid, m_hotList) {
					activateOutput(uuid);
				}
			}
			m_isFirstRun = false;
			emit scriptResumed();
		}
		else if (value == "pause")
			emit scriptPaused();
		else if (value == "quit") {
			emit scriptStopped();
			// Restore the first run status
			m_isFirstRun = true;
		}
		else
			emit displayStatusMessage(tr("Unknown script status \"") + value + tr("\" received."),
		                              MSG_WARNING);
	}
}
