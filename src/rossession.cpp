#include "rossession.h"

#include <QDebug>

#include "hieroglyph/SimpleCmd.h"

ROSSession::ROSSession(QObject *parent) : QObject(parent),
                           m_nodeName(QString()),
                           m_isConnected(false),
                           m_isRunning(false),
                           m_isPaused(false),
                           m_timeOffset(0),
                           m_startTime()
{
    startTimer(83); // Not a round number so that the hundresth of a second digit doesn't stay the same
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
    // TODO: emit message for status bar
    if (!m_isRunning)
        return;

    if (m_isPaused)
        run();
    else
        pause();
}

void ROSSession::run()
{
    // TODO emit message for status bar
    if (m_nodeName.isEmpty()) {
        qWarning() << "No node name: cannot run";
        return;
    }

    QString srvName = m_nodeName + "/control";
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
        qDebug() << "Failed to call RUN";
    }
}

void ROSSession::pause()
{
    // TODO emit message for status bar
    if (m_nodeName.isEmpty()) {
        qWarning() << "No node name: cannot pause";
        return;
    }

    QString srvName = m_nodeName + "/control";
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
        qWarning() << "No node name: cannot pause";
        return;
    }

    QString srvName = m_nodeName + "/control";
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
        qDebug() << "Failed to call STOP";
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
