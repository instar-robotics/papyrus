#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

//#include "script.h"
#include "types.h"

#include <QObject>
#include <QString>
#include <QDateTime>

// Forward definition of the Script class
class Script;

enum ScriptStatus {
    SCRIPT_RUNNING,
    SCRIPT_PAUSED
};

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QObject
{
    Q_OBJECT

public:
    ROSSession(QObject *parent = nullptr, Script *script = NULL);
    ~ROSSession();

    void timerEvent(QTimerEvent *evt);

    void runOrPause();
    void run();
    void pause();
    void stop();
    ScriptStatus queryScriptStatus();

    QString nodeName() const;
    void setNodeName(const QString &nodeName);

    bool isRunning() const;
    void setIsRunning(bool isRunning);

    bool isPaused() const;
    void setIsPaused(bool isPaused);

private:
    QString m_nodeName;       // the ROS node we are connected to
    ros::NodeHandle m_n;      // The ROS handle through which we will issue commands
    bool m_isRunning;         // indicates whether the associated kheops script is currently running
    bool m_isPaused;          // indicated whether the associated kheops script is paused
    qint64 m_timeOffset;      // time accumulator to keep track of elapsed time when script is paused
    QDateTime m_startTime;    // start time of the last "run"
    Script *m_script;         // the script to which this session is asscoiated

signals:
    void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
    void scriptResumed();     // emited when "play" action succeeded
    void scriptPaused();      // emited when "pause" action succeeded
    void scriptStopped();     // emitted when "stop" action succeeded
    void timeElapsed(int h, int m, int s, int ms); // emited regularly with the new run time
};

#endif // ROSSESSION_H
