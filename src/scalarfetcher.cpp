#include "scalarfetcher.h"
#include "scalarvisualization.h"

#include <QDebug>

ScalarFetcher::ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent) :
    DataFetcher(topicName, parent),
    m_scalarVisualization(scalarVisualization)
{
	qDebug() << "[ScalarFetcher] created on topic" << m_topicName;

	qDebug() << "[ScalarFetcher] start";
	start();
}

ScalarFetcher::ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent) :
    ScalarFetcher(topicName, scalarVisualization, parent)
{
	setVisType(type);
}

void ScalarFetcher::fetchScalar(const std_msgs::Float64::ConstPtr &scalar)
{
	switch (m_visType) {
		case BAR:
			if (m_scalarVisualization != nullptr) {
//				m_scalarVisualization->updateBarValue(scalar->data);
				std::vector<qreal> v;
				v.push_back(scalar->data);
				m_scalarVisualization->updateBarValues(v);
			}
		break;

		case GRAPH:
			if (m_scalarVisualization != nullptr) {
//				m_scalarVisualization->pushGraphValue(scalar->data);
				std::vector<qreal> v;
				v.push_back(scalar->data);
				m_scalarVisualization->pushGraphValues(v);
			}
		break;

		default:
			qWarning() << "ScalarFetcher cannot deal with type" << m_visType;
		break;
	}
}

/**
 * @brief ScalarFetcher::setVisType changes the visualization type if it matches the allowed ones
 * for this fetcher
 * @param type
 */
void ScalarFetcher::setVisType(VisualizationType type)
{
	if (type == m_visType)
		return;

	if (type <= GRAPH)
		m_visType = type;
	else
		qWarning() << "[ScalarFetcher] ignores new visualization type" << type;
}

void ScalarFetcher::run()
{
	qDebug() << "[ScalarFetcher] run";
	ros::Rate rate(10); // 10Hz

	ros::Subscriber m_sub = m_n.subscribe(m_topicName.toStdString(),
	                                      1000,
	                                      &ScalarFetcher::fetchScalar, this);

	while (ros::ok()) {
		if (m_shouldQuit) {
			qDebug() << "[ScalarFetcher] quitting";
			quit();
			return;
		}

		ros::spinOnce();
		rate.sleep();
	}
}
