#ifndef ROSSESSION_H
#define ROSSESSION_H

#include <ros/ros.h>

//#include "script.h"
#include "types.h"

#include <QThread>
#include <QString>
#include <QSet>

#include "diagnostic_msgs/KeyValue.h"

/**
 * @brief The ROSSession class contains parameters related to the current ROS session (connection
 * with a Kheops node
 */
class ROSSession : public QThread
{
	Q_OBJECT

public:
	ROSSession(const QString &nodeName, QObject *parent = nullptr);
	~ROSSession();

	void addToHotList(QUuid uuid);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	QString nodeName() const;
	void setNodeName(const QString &nodeName);

private:
	bool m_shouldQuit;       // Used to properly exit the thread
	QString m_nodeName;      // Name of the node it should listen
	QSet<QUuid> m_hotList;   // List of functions' uuids whose output to activate on-the-fly
	bool m_isFirstRun;       // To differentiate a 'resume' on first launch or after a pause

	void run() override;
	void handleStatusChange(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	void activateOutput(QUuid uuid);

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void scriptResumed();
	void scriptPaused();
	void scriptStopped();
};

#endif // ROSSESSION_H
