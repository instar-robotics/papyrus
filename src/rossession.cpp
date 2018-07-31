#include "rossession.h"

#include <QDebug>

#include "hieroglyph/SimpleCmd.h"

ROSSession::ROSSession(QObject *parent) : QObject(parent),
                           m_nodeName(QString()),
                           m_isConnected(false),
                           m_isRunning(false),
                           m_isPaused(false)
{

}

ROSSession::~ROSSession()
{

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
            emit scriptPaused();
        }
    } else {
        qDebug() << "Failed to call PAUSE";
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
