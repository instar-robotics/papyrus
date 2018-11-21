#include "rossession.h"
#include "helpers.h"
#include "papyruswindow.h"

ROSSession::ROSSession(const QString &nodeName, QObject *parent)
    : QThread(parent),
      m_shouldQuit(false),
      m_nodeName(nodeName)
{
	start();
}

ROSSession::~ROSSession()
{

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

	ros::Rate rate(10); // 10Hz
	ros::NodeHandle nh;

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
		if (value == "resume")
			emit scriptResumed();
		else if (value == "pause")
			emit scriptPaused();
		else if (value == "quit")
			emit scriptStopped();
		else
			emit displayStatusMessage(tr("Unknown script status \"") + value + tr("\" received."),
		                              MSG_WARNING);
	}
}
