#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

//#include "script.h"
#include "types.h"

#include <QThread>
#include <QString>

#include "diagnostic_msgs/KeyValue.h"

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QThread
{
	Q_OBJECT

public:
	ROSSession(const QString &topicName, QObject *parent = nullptr);
	~ROSSession();

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

private:
	bool m_shouldQuit;       // Used to properly exit the thread
	QString m_nodeName;      // Name of the node it should listen

	void run() override;
	void handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr& msg);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
};

#endif // ROSSESSION_H
