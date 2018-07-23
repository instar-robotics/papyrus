#include "rosnode.h"

#include <QDebug>

#include "std_msgs/String.h"


RosNode::RosNode(int argc, char **argv) : m_argc(argc), m_argv(argv)
{
}

RosNode::~RosNode()
{
    qDebug() << "[DESTRUCTOR]";

    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }

    wait();
}

void cb1(const std_msgs::String::ConstPtr& msg)
{
    qDebug() << "cb1 called, data: " << msg->data.c_str();
}

bool RosNode::init()
{
    // Initialize the ROS API (must be the first ROS-related call)
    ros::init(m_argc, m_argv, "Papyrus");

    // Start the QThread (we need this so that waiting for the ROS master is done in thread)
    start();
}

void RosNode::run()
{
    // Wait for the master node to come online
    while (!ros::master::check()) {
        qDebug() << "Waiting for ROS master to spawn...";
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
    while (ros::ok() && ros::master::check()) {
        ros::spinOnce();
    }

    emit rosMasterChanged(false);
//    ros::spin();

    qDebug() << "ROS SPIN OVER";
}

