#include "datafetcher.h"

#include <QDebug>

DataFetcher::DataFetcher(const QString &topicName, QObject *parent) :
    QThread(parent),
    m_topicName(topicName),
    m_shouldQuit(false)
{
	qDebug() << "[DataFetcher] created on topic" << m_topicName;

//	m_sub = m_n.subscribe(topicName.toStdString(), 1000, &DataFetcher::fetchScalar, this);
}

// TODO: define a loop rate instead of spinning like hell
/*
void DataFetcher::run()
{
	qDebug() << "[DataFetcher] Run";
//	ros::Subscriber m_sub = m_n.subscribe(m_topicName.toStdString(), 1000, &DataFetcher::fetchScalar, this);

	while (ros::ok()) {
		if (m_shouldQuit) {
			qDebug() << "[DataFetcher] should quit now";
			quit();
			return;
		}

		ros::spinOnce();
	}
}
//*/

/*
void DataFetcher::fetchScalar(const std_msgs::Float64::ConstPtr &scalar)
{
//	qDebug() << "[FetchScalar] called with value:" << scalar->data;
	qDebug() << "Replacing";
	m_barSet->replace(0, scalar->data);
	qDebug() << "Done replacing";
}
//*/

bool DataFetcher::shouldQuit() const
{
	return m_shouldQuit;
}

void DataFetcher::setShouldQuit(bool shouldQuit)
{
	m_shouldQuit = shouldQuit;
}

VisualizationType DataFetcher::visType() const
{
	return m_visType;
}
