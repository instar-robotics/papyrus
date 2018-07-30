#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <QString>

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession {

public:
    ROSSession();
    ~ROSSession();

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
    bool m_isConnected;       // indicates whether we are connected to a kheops node
    bool m_isRunning;         // indicates whether the associated kheops script is currently running
    bool m_isPaused;          // indicated whether the associated kheops script is paused
};

#endif // ROSSESSION_H
