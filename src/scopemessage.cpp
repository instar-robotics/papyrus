#include "scopemessage.h"

ScopeMessage::ScopeMessage()
{

}

ScopeMessage::ScopeMessage(const ScopeMessage &other) :
    m_uuid(other.uuid()),
    m_means(other.means()),
    m_duration(other.duration()),
    m_start(other.start()),
    m_minDuration(other.minDuration()),
    m_maxDuration(other.maxDuration())
{

}

ScopeMessage::~ScopeMessage()
{

}

QUuid ScopeMessage::uuid() const
{
	return m_uuid;
}

void ScopeMessage::setUuid(const QUuid &uuid)
{
	m_uuid = uuid;
}

qreal ScopeMessage::means() const
{
	return m_means;
}

void ScopeMessage::setMeans(const qreal &means)
{
	m_means = means;
}

qreal ScopeMessage::duration() const
{
	return m_duration;
}

void ScopeMessage::setDuration(const qreal &duration)
{
	m_duration = duration;
}

qreal ScopeMessage::start() const
{
	return m_start;
}

void ScopeMessage::setStart(const qreal &start)
{
	m_start = start;
}

qreal ScopeMessage::minDuration() const
{
	return m_minDuration;
}

void ScopeMessage::setMinDuration(const qreal &minDuration)
{
	m_minDuration = minDuration;
}

qreal ScopeMessage::maxDuration() const
{
	return m_maxDuration;
}

void ScopeMessage::setMaxDuration(const qreal &maxDuration)
{
	m_maxDuration = maxDuration;
}
