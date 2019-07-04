#include "rttokenmessage.h"

RTTokenMessage::RTTokenMessage()
{

}

RTTokenMessage::RTTokenMessage(const RTTokenMessage &copy)
    : m_uuid(copy.m_uuid)
    , m_period(copy.m_period)
    , m_means(copy.m_means)
    , m_sleep(copy.m_sleep)
    , m_duration(copy.m_duration)
    , m_start(copy.m_start)
    , m_minDuration(copy.m_minDuration)
    , m_maxDuration(copy.m_maxDuration)
    , m_warning(copy.m_warning)
{

}

RTTokenMessage::~RTTokenMessage()
{

}

QUuid RTTokenMessage::uuid() const
{
	return m_uuid;
}

void RTTokenMessage::setUuid(const QUuid &uuid)
{
	m_uuid = uuid;
}

qreal RTTokenMessage::period() const
{
	return m_period;
}

void RTTokenMessage::setPeriod(const qreal &period)
{
	m_period = period;
}

qreal RTTokenMessage::means() const
{
	return m_means;
}

void RTTokenMessage::setMeans(const qreal &means)
{
	m_means = means;
}

qreal RTTokenMessage::sleep() const
{
	return m_sleep;
}

void RTTokenMessage::setSleep(const qreal &sleep)
{
	m_sleep = sleep;
}

qreal RTTokenMessage::duration() const
{
	return m_duration;
}

void RTTokenMessage::setDuration(const qreal &duration)
{
	m_duration = duration;
}

qreal RTTokenMessage::start() const
{
	return m_start;
}

void RTTokenMessage::setStart(const qreal &start)
{
	m_start = start;
}

qreal RTTokenMessage::minDuration() const
{
	return m_minDuration;
}

void RTTokenMessage::setMinDuration(const qreal &minDuration)
{
	m_minDuration = minDuration;
}

qreal RTTokenMessage::maxDuration() const
{
	return m_maxDuration;
}

void RTTokenMessage::setMaxDuration(const qreal &maxDuration)
{
	m_maxDuration = maxDuration;
}

bool RTTokenMessage::warning() const
{
	return m_warning;
}

void RTTokenMessage::setWarning(bool warning)
{
	m_warning = warning;
}
