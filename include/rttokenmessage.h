#ifndef RTTOKENMESSAGE_H
#define RTTOKENMESSAGE_H

#include <QMetaType>
#include <QUuid>

/**
 * @brief The RTTokenMessage class represents a topic message from 'rt_token' kheops topic.
 * Since we intend to pass this information through signals, we have to make a custom class with
 * specific implementation to be able to use it as params for signals/slots.
 */
class RTTokenMessage
{
public:
	RTTokenMessage();
	RTTokenMessage(const RTTokenMessage& copy);
	~RTTokenMessage();

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	qreal period() const;
	void setPeriod(const qreal &period);

	qreal means() const;
	void setMeans(const qreal &means);

	qreal sleep() const;
	void setSleep(const qreal &sleep);

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
	QUuid m_uuid;        // RTToken's uuid
	qreal m_period;      // Period of the script
	qreal m_means;       // Average running time of the script since its launch
	qreal m_sleep;       // Time (in s) the RT Token slept at the end of the execution
	qreal m_duration;    // Duration (in s) this execution took
	qreal m_start;       // Timestamp where this execution began
	qreal m_minDuration; // Quickest execution so far
	qreal m_maxDuration; // Slowest execution so far
	bool m_warning;      // Wether this execution failed to hold real time constraints
};

Q_DECLARE_METATYPE(RTTokenMessage)

#endif // RTTOKENMESSAGE_H
