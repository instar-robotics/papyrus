#include "rosnode.h"

#include <QDebug>

#include "std_msgs/String.h"


RosNode::RosNode(int argc, char **argv) : m_argc(argc), m_argv(argv), m_shouldQuit(false)
{
}

RosNode::~RosNode()
{
	if(ros::isStarted()) {
		qDebug() << "ROS Node thread terminating ROS";
		ros::shutdown();
		ros::waitForShutdown();
	}
}

void cb1(const std_msgs::String::ConstPtr& msg)
{
	qDebug() << "cb1 called, data: " << msg->data.c_str();
}

void RosNode::init()
{
	// Initialize the ROS API (must be the first ROS-related call)
//    ros::init(m_argc, m_argv, "Papyrus");

	// Start the QThread (we need this so that waiting for the ROS master is done in thread)
	start();
}

void RosNode::run()
{
	// Wait for the master node to come online
	while (!ros::master::check()) {
		sleep(1);
	}

	// Emit the signal telling the GUI the ROS master came online
	emit rosMasterChanged(true);

	// We start ros manually because the node handle will go out of scope
	ros::start();

	// Create a subscriber
	ros::NodeHandle n;
	m_sub = n.subscribe("chatter", 1000, cb1);

	// And now enter the ROS loop
	ros::Rate rate(10); // 10 Hz
	while (ros::ok() && ros::master::check()) {
		if (m_shouldQuit) {
			quit();
			return;
		}

		ros::spinOnce();
		rate.sleep();
	}

	emit rosMasterChanged(false);
}

bool RosNode::shouldQuit() const
{
	return m_shouldQuit;
}

void RosNode::setShouldQuit(bool value)
{
	m_shouldQuit = value;
}

