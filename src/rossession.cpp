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
                           m_isConnected(false),
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
    // TODO emit message for status bar
    if (m_nodeName.isEmpty()) {
        emit displayStatusMessage(tr("No script name: cannot run"), MSG_ERROR);
        return;
    }

    // if the name is not running (was stopped), we need to get its name. Either because we have an
    // associated Script, or else we ask the user to provide the path to the file
    // this is TEMPORARY, waiting for kheops/#11 to be solved
    if (!m_isRunning) {
        QString scriptPath;

        // If we are connected to the current script (and there's one), fetch its script path from
        // its 'Script' object
        if (m_nodeName == "Current Script") {
            // First check that we have at least one script opened and active (this is temporary
            // because later, a ROSSession will be attached to a Script)
            PapyrusWindow *mainWin = getMainWindow();

            if (mainWin->activeScript() != NULL) {
                scriptPath = mainWin->activeScript()->filePath();

                if (scriptPath.isEmpty()) {
                    emit displayStatusMessage(tr("No filepath for the current script yet, please "
                                                 "save at least once and retry!"), MSG_WARNING);
                    return;
                }
            } else {
                emit displayStatusMessage(tr("No current script active!"), MSG_WARNING);
                return;
            }

        } else {
            QString home = QProcessEnvironment::systemEnvironment().value("HOME", "/home");
            scriptPath = QFileDialog::getOpenFileName(NULL, tr("Script file"), home, tr("XML script files (*.xml, *.XML)"));
        }

        if (!scriptPath.isEmpty()) {
            QProcess *kheopsNode = new QProcess(this);
            QString prog = "rosrun";
            QStringList args;
            args << "kheops";
            args << "kheops";
            args << "-r"; // we start the node directly in run mode
            args << "-s";
            args << scriptPath;
            kheopsNode->start(prog, args);

            // Note that it just means the program was started, but it may as well crash just after launch
            if (kheopsNode->waitForStarted(2000)) {
                m_isRunning = true;
                m_isPaused = false;

                // Record a new starting time for the run
                m_startTime = QDateTime::currentDateTime();

                emit scriptResumed();
            } else {
                emit displayStatusMessage(tr("Failed to launch script."), MSG_ERROR);
                qWarning() << "Failed to re-launch with cmd \"" << prog << args << "\"";
            }
        } else {
            emit displayStatusMessage(tr("No script path: cannot launch."), MSG_WARNING);
            qWarning() << "Empty script path: cannot re-launch";
        }
    } else {
        QString nodeName;

        // If the node is the current script, we have to craft its name from the Script's attribute
        if (m_nodeName == "Current Script") {
            // First check that we have at least one script opened and active (this is temporary
            // because later, a ROSSession will be attached to a Script)
            PapyrusWindow *mainWin = getMainWindow();

            if (mainWin->activeScript() != NULL) {
                QString scriptName = mainWin->activeScript()->name();

                if (scriptName.isEmpty()) {
                    emit displayStatusMessage(tr("Script has no name yet, please save it with a name and retry."), MSG_WARNING);
                    return;
                }

                nodeName = "/kheops_" + scriptName;
            } else {
                emit displayStatusMessage(tr("No current script active!"), MSG_WARNING);
                return;
            }

        }
        // Otherwise just use the saved node name
        else {
            nodeName = m_nodeName;
        }

        QString srvName = nodeName + "/control";
        ros::ServiceClient client = m_n.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
        hieroglyph::SimpleCmd srv;
        srv.request.cmd = "resume";

        if (client.call(srv)) {
            QString response = QString::fromStdString(srv.response.ret);;

            if (response == "resume") {
                m_isRunning = true;
                m_isPaused = false;

                // Record a new starting time for the run
                m_startTime = QDateTime::currentDateTime();

                emit scriptResumed();
            }
        } else {
            emit displayStatusMessage(tr("The RUN command failed."), MSG_ERROR);
            qDebug() << "Failed to call RUN";
        }
    }
}

void ROSSession::pause()
{
    // TODO emit message for status bar
    if (m_nodeName.isEmpty()) {
        qWarning() << "No node name: cannot pause";
        return;
    }

    QString nodeName;

    // If the node is the current script, we have to craft its name from the Script's attribute
    if (m_nodeName == "Current Script") {
        // First check that we have at least one script opened and active (this is temporary
        // because later, a ROSSession will be attached to a Script)
        PapyrusWindow *mainWin = getMainWindow();

        if (mainWin->activeScript() != NULL) {
            QString scriptName = mainWin->activeScript()->name();

            if (scriptName.isEmpty()) {
                emit displayStatusMessage(tr("Script has no name yet, please save it with a name and retry."), MSG_WARNING);
                return;
            }

            nodeName = "/kheops_" + scriptName;
        } else {
            emit displayStatusMessage(tr("No current script active!"), MSG_WARNING);
            return;
        }

    }
    // Otherwise just use the saved node name
    else {
        nodeName = m_nodeName;
    }

    QString srvName = nodeName + "/control";
    ros::ServiceClient client = m_n.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
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
    // TODO: emit message for status bar
    if (!m_isRunning)
        return;

    // TODO emit message for status bar
    if (m_nodeName.isEmpty()) {
        emit displayStatusMessage(tr("No node name for this script: cannot stop"), MSG_ERROR);
        qWarning() << "No node name: cannot pause";
        return;
    }

    QString nodeName;

    // If the node is the current script, we have to craft its name from the Script's attribute
    if (m_nodeName == "Current Script") {
        // First check that we have at least one script opened and active (this is temporary
        // because later, a ROSSession will be attached to a Script)
        PapyrusWindow *mainWin = getMainWindow();

        if (mainWin->activeScript() != NULL) {
            QString scriptName = mainWin->activeScript()->name();

            if (scriptName.isEmpty()) {
                emit displayStatusMessage(tr("Script has no name yet, please save it with a name and retry."), MSG_WARNING);
                return;
            }

            nodeName = "/kheops_" + scriptName;
        } else {
            emit displayStatusMessage(tr("No current script active!"), MSG_WARNING);
            return;
        }

    }
    // Otherwise just use the saved node name
    else {
        nodeName = m_nodeName;
    }

    QString srvName = nodeName + "/control";
    ros::ServiceClient client = m_n.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
    hieroglyph::SimpleCmd srv;
    srv.request.cmd = "quit";

    // TODO: remove 'true', when kheops/#10 is solved
    if (client.call(srv) || true) {
        QString response = QString::fromStdString(srv.response.ret);;

        // TODO: same here
        if (true || response == "quit") {
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
    if (!m_isConnected)
        informUserAndCrash(tr("Cannot query for script's status because not connected to a script."),
                           tr("Not connected to a script"));

    if (m_nodeName.isEmpty())
        informUserAndCrash(tr("Cannot query for script's status because no node name was specified."),
                           tr("No node name specified"));

    if (m_nodeName == "Current Script")
        informUserAndCrash(tr("Cannot query a \"Current Script\" for its status."));

    QString srvName = m_nodeName + "/control";

    ros::ServiceClient client = m_n.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
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
}

bool ROSSession::isConnected() const
{
    return m_isConnected;
}

void ROSSession::setIsConnected(bool isConnected)
{
    m_isConnected = isConnected;
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
