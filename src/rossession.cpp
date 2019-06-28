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
#include "hieroglyph/SimpleCmd.h"

ROSSession::ROSSession(const QString &nodeName, QObject *parent)
    : QThread(parent),
      m_nh(nullptr),
      m_shouldQuit(false),
      m_nodeName(nodeName),
      m_isFirstRun(true)
{
	m_nh = new ros::NodeHandle;
	start();
}

ROSSession::~ROSSession()
{
	callServiceOscillo("stop");

	if (m_nh != nullptr) {
		delete m_nh;
		m_nh = nullptr;
	}
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
 * @brief ROSSession::callServiceControl calls the ROS service /control of kheops
 * @return
 */
bool ROSSession::callServiceControl(QString cmd)
{
//	ros::NodeHandle nh;
	QString srvName = QString("%1/control").arg(m_nodeName);

	ros::ServiceClient client = m_nh->serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = cmd.toStdString();

	return client.call(srv);
}

ScriptStatus ROSSession::queryScriptStatus()
{
	QString srvName = QString("%1/control").arg(m_nodeName);

//	ros::NodeHandle nh;
	ros::ServiceClient client = m_nh->serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = "status";

	if (client.call(srv)) {
		QString response = QString::fromStdString(srv.response.ret);

		if (response == "run")
			return SCRIPT_RUNNING;
		else if (response == "pause")
			return SCRIPT_PAUSED;
		else
			return INVALID_SCRIPT_STATUS;
	} else {
		return INVALID_SCRIPT_STATUS;
	}

	return INVALID_SCRIPT_STATUS;
}

/**
 * @brief ROSSession::callServiceOscillo calls the ROSService "oscillo" with the given command.
 * @param cmd the command to pass to the service, supported are: "start", "stop"
 * @return whether the call was successful
 */
bool ROSSession::callServiceOscillo(const QString &cmd)
{
//	ros::NodeHandle nh;
	QString srvName = QString("%1/oscillo").arg(m_nodeName);

	ros::ServiceClient client = m_nh->serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = cmd.toStdString();

	return client.call(srv);
}

void ROSSession::registerOscillo()
{
	// Subscribe to the 'oscillo' topic to listen to status change
	m_subs << m_nh->subscribe(QString("%1/%2").arg(m_nodeName, "oscillo").toStdString(),
	                                   1,
	                                   &ROSSession::handleOscilloMessage,
	                                   this);
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
//	ros::NodeHandle nh;
	ros::ServiceClient client = m_nh->serviceClient<hieroglyph::ArgCmd>(srvName.toStdString());
	hieroglyph::ArgCmd srv;
	srv.request.cmd = "start";
	srv.request.arg = uuid.toString().toStdString();

	if (!client.call(srv)) {
		qWarning() << "Failed to activate output on uuid" << uuid.toString() << "on node name" << m_nodeName;
	}
}

void ROSSession::handleOscilloMessage(const hieroglyph::OscilloArray::ConstPtr &msg)
{
	QVector<ScopeMessage> *scopeMessages = new QVector<ScopeMessage>;

	int n = msg->array.size();

	for (int i = 0; i < n; i += 1) {
		ScopeMessage message;
		hieroglyph::OscilloData data = msg->array.at(i);

		message.setUuid(QUuid(data.uuid.c_str()));
		message.setPeriod(data.period);
		message.setMeans(data.means);
		message.setDuration(data.duration);
		message.setStart(data.start);
		message.setMinDuration(data.minDuration);
		message.setMaxDuration(data.maxDuration);
		message.setWarning(data.warning);

		*scopeMessages << message;
	}

	emit newOscilloMessage(scopeMessages);
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

//	ros::NodeHandle nh;
	ros::Rate rate(10); // 10Hz

	// Subscribe to the 'status' topic to listen to status change
	ros::Subscriber sub = m_nh->subscribe(QString("%1/%2").arg(m_nodeName, "status").toStdString(),
	                                   10,
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
