#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

#include <QObject>
#include <QString>
#include <QDateTime>

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

    void timerEvent(QTimerEvent *evt);

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
    qint64 m_timeOffset;      // time accumulator to keep track of elapsed time when script is paused
    QDateTime m_startTime;    // start time of the last "run"

signals:
    void scriptResumed();     // emited when "play" action succeeded
    void scriptPaused();      // emited when "pause" action succeeded
    void timeElapsed(int h, int m, int s, int ms); // emited regularly with the new run time
};

#endif // ROSSESSION_H
