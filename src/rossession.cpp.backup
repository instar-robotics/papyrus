#include "rossession.h"
#include "helpers.h"
#include "papyruswindow.h"

#include <QDebug>
#include <QFileDialog>
#include <QProcessEnvironment>
#include <QProcess>

#include "hieroglyph/SimpleCmd.h"

ROSSession::ROSSession(QObject *parent, Script *script) : QObject(parent),
                           m_nodeName(QString()),
                           m_isRunning(false),
                           m_isPaused(false),
                           m_timeOffset(0),
                           m_startTime(),
                           m_script(script)
{
	startTimer(83); // Not a round number so that the hundredth of a second digit doesn't stay the same
}

ROSSession::~ROSSession()
{

}

/**
 * @brief ROSSession::timerEvent is used to increment the run time clock of the script, if it is
 * running and not paused
 * @param evt
 */
void ROSSession::timerEvent(QTimerEvent *evt)
{
	Q_UNUSED(evt);

	if (m_isRunning && !m_isPaused) {
		qint64 ms = m_timeOffset +  m_startTime.msecsTo(QDateTime::currentDateTime());
		int h = ms / 1000 / 60 / 60;
		int m = (ms / 1000 / 60) - (h * 60);
		int s = (ms / 1000) - (m * 60);
		ms = ms - (s * 1000);
		emit timeElapsed(h, m, s, ms);
	}
}

void ROSSession::runOrPause()
{
	if (m_isRunning && !m_isPaused)
		pause();
	else
		run();
}

void ROSSession::run()
{
	// Make sure the ROS master is up before trying to run
	if (!ros::master::check()) {
		emit displayStatusMessage(tr("Command cancelled because the ROS master is not up."), MSG_ERROR);
		return;
	}

	if (m_nodeName.isEmpty()) {
		emit displayStatusMessage(tr("No node name: cannot run"), MSG_ERROR);
		return;
	}

	// Save the node before launching it
	PapyrusWindow *mainWin = getMainWindow();
	m_script->save(mainWin->getDescriptionPath(), mainWin->lastDir());

	// Check if the save worked
	if (m_script->modified()) {
		emit displayStatusMessage(tr("You need to provide a file to save the script in order to run it"),
		                          MSG_WARNING);
		return;
	}

	// If the node is not running, we need to launch a kheops instance
	if (!m_isRunning) {
		QProcess *kheopsNode = new QProcess(this);
		//*
		QString prog = "rosrun";
		QStringList args;
		args << "kheops";
		args << "kheops";
		//*/
		// TEMPORARY
		/*
		QString prog = "/home/nschoe/workspace/Qt/catkin_ws/devel/lib/kheops/kheops";
		QStringList args;
		//*/
		args << "-s";
		args << m_script->filePath();
		args << "-l";
		args << mainWin->getLibPath() + "/";
		emit displayStatusMessage(tr("Starting script \"") + m_script->name() + "\"...");
		qDebug() << "[RUN]" << prog << args;
		kheopsNode->start(prog, args);
	}
	// Otherwise, if the node is already running, we just have to ask it to resume execution
	else {
		ros::NodeHandle nh;
		QString srvName = m_nodeName + "/control";
		ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
		hieroglyph::SimpleCmd srv;
		srv.request.cmd = "resume";

		// We switched to asynchronous mode: so discard the answer to this ROS service
		// instead, the status will be reported by the "/status" topic
		/*
		if (client.call(srv)) {
			QString response = QString::fromStdString(srv.response.ret);

			if (response == "resume") {
				m_isRunning = true;
				m_isPaused = false;

				// Record a new starting time for the run
				m_startTime = QDateTime::currentDateTime();

				emit scriptResumed();
			}
			*/

		emit displayStatusMessage(tr("Resuming script \"") + m_script->name() + "\"...");
		if (!client.call(srv)) {
			emit displayStatusMessage(tr("The RUN command failed."), MSG_ERROR);
		}
	}
}

void ROSSession::pause()
{
	if (m_nodeName.isEmpty()) {
		emit displayStatusMessage(tr("No node name: cannot pause"), MSG_ERROR);
		qWarning() << "No node name: cannot pause";
		return;
	}

	ros::NodeHandle nh;

	QString srvName = m_nodeName + "/control";
	ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = "pause";

	if (client.call(srv)) {
		QString response = QString::fromStdString(srv.response.ret);;

		if (response == "pause") {
			m_isRunning = true;
			m_isPaused = true;

			// Update the time offset
			m_timeOffset += m_startTime.msecsTo(QDateTime::currentDateTime());

			emit scriptPaused();
		}
	} else {
		emit displayStatusMessage(tr("The PAUSE command failed."), MSG_ERROR);
		qDebug() << "Failed to call PAUSE";
	}
}

void ROSSession::stop()
{
	// Make sure the ROS master is up
	if (!ros::master::check()) {
		emit displayStatusMessage(tr("STOP command cancelled because the ROS master is not up."), MSG_ERROR);
		return;
	}

	ros::NodeHandle nh;

	if (m_nodeName.isEmpty()) {
		emit displayStatusMessage(tr("No node name for this script: cannot stop"), MSG_ERROR);
		qWarning() << "No node name: cannot stop";
		return;
	}

	QString srvName = m_nodeName + "/control";

	ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = "quit";

	if (client.call(srv)) {
		QString response = QString::fromStdString(srv.response.ret);;

		if (response == "quit") {
			m_isRunning = false;
			m_isPaused = false;

			// Reset the time offset
			m_timeOffset = 0;

			emit scriptStopped();
		}
	} else {
		emit displayStatusMessage(tr("The STOP command failed."), MSG_ERROR);
		qDebug() << "Failed to call STOP";
	}
}

/**
 * @brief ROSSession::queryScriptStatus makes a ROS service call to the "control" endpoint, with the
 * "status" parameters to actually query the script for its status.
 * @return
 */
ScriptStatus ROSSession::queryScriptStatus()
{
	// Make sure the ROS master is up
	if (!ros::master::check()) {
		emit displayStatusMessage(tr("Command cancelled because the ROS master is not up."), MSG_ERROR);
		return INVALID_SCRIPT_STATUS;
	}

	ros::NodeHandle nh;

	if (m_nodeName.isEmpty())
		informUserAndCrash(tr("Cannot query for script's status because no node name was specified."),
		                   tr("No node name specified"));

	QString srvName = m_nodeName + "/control";

	ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = "status";

	if (client.call(srv)) {
		QString response = QString::fromStdString(srv.response.ret);;

		if (response == "run")
			return SCRIPT_RUNNING;
		else if (response == "pause")
			return SCRIPT_PAUSED;
		else
			informUserAndCrash(tr("Invalid data \"") + response + tr(" returned from /control status.\n"
			                                                         "it might be due to an API change or a malfunction"),
			                   tr("Invalid return data type"));
	} else {
		informUserAndCrash(tr("The command STATUS failed when the node was queried."),
		                   tr("Failed STATUS command"));
	}

	return INVALID_SCRIPT_STATUS;
}

bool ROSSession::isRunning() const
{
	return m_isRunning;
}

void ROSSession::setIsRunning(bool isRunning)
{
	m_isRunning = isRunning;
}

bool ROSSession::isPaused() const
{
	return m_isPaused;
}

void ROSSession::setIsPaused(bool isPaused)
{
	m_isPaused = isPaused;
}

QString ROSSession::nodeName() const
{
	return m_nodeName;
}

void ROSSession::setNodeName(const QString &nodeName)
{
	m_nodeName = nodeName;
}
