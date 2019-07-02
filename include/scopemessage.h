#ifndef SCOPEMESSAGE_H
#define SCOPEMESSAGE_H

#include <QUuid>
#include <QMetaType>

class ScopeMessage
{
public:
	ScopeMessage();
	ScopeMessage(const ScopeMessage& other);
	~ScopeMessage();

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	qreal period() const;
	void setPeriod(const qreal &period);

	qreal means() const;
	void setMeans(const qreal &means);

	qreal duration() const;
	void setDuration(const qreal &duration);

	qreal start() const;
	void setStart(const qreal &start);

	qreal minDuration() const;
	void setMinDuration(const qreal &minDuration);

	qreal maxDuration() const;
	void setMaxDuration(const qreal &maxDuration);

	bool warning() const;
	void setWarning(bool warning);

private:
	QUuid m_uuid;
	qreal m_period;
	qreal m_means;
	qreal m_duration;
	qreal m_start;
	qreal m_minDuration;
	qreal m_maxDuration;
	bool m_warning;
};

Q_DECLARE_METATYPE(ScopeMessage)

#endif // SCOPEMESSAGE_H
