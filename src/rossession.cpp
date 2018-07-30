#include "rossession.h"

#include <QDebug>

ROSSession::ROSSession() : m_nodeName(QString()),
                           m_isConnected(false),
                           m_isRunning(false),
                           m_isPaused(false)
{

}

ROSSession::~ROSSession()
{

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
