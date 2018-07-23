#ifndef ROSNODE_H
#define ROSNODE_H

#include <QThread>

#include <ros/ros.h>

/**
 * @brief The RosNode class is used to handle ROS
 * operations
 */
class RosNode : public QThread
{
    Q_OBJECT
public:
    RosNode(int argc, char **argv);
    virtual ~RosNode();

    bool init();
    void run() override;

private:
    int m_argc;
    char **m_argv;
    ros::Subscriber m_sub;

signals:
    void rosMasterChanged(bool isOnline);
};

#endif // ROSNODE_H
