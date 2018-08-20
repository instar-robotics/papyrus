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

    bool shouldQuit() const;
    void setShouldQuit(bool value);

private:
    int m_argc;
    char **m_argv;
    ros::Subscriber m_sub;
    bool m_shouldQuit;       // Used by Papyrus to cleanly exit the thread

signals:
    void rosMasterChanged(bool isOnline);
};

#endif // ROSNODE_H
