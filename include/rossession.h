#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

#include <QObject>
#include <QString>

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QObject
{
    Q_OBJECT

public:
    ROSSession(QObject *parent = nullptr);
    ~ROSSession();

    void runOrPause();
    void run();
    void pause();

    QString nodeName() const;
    void setNodeName(const QString &nodeName);

    bool isConnected() const;
    void setIsConnected(bool isConnected);

    bool isRunning() const;
    void setIsRunning(bool isRunning);

    bool isPaused() const;
    void setIsPaused(bool isPaused);

private:
    QString m_nodeName;       // the ROS node we are connected to
    ros::NodeHandle m_n;      // The ROS handle through which we will issue commands
    bool m_isConnected;       // indicates whether we are connected to a kheops node
    bool m_isRunning;         // indicates whether the associated kheops script is currently running
    bool m_isPaused;          // indicated whether the associated kheops script is paused

signals:
    void scriptResumed();     // emited when "play" action succeeded
    void scriptPaused();      // emited when "pause" action succeeded
};

#endif // ROSSESSION_H
