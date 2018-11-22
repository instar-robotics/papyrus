#include "scalarfetcher.h"
#include "scalarvisualization.h"

#include <QDebug>

ScalarFetcher::ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent) :
    DataFetcher(topicName, parent),
    m_scalarVisualization(scalarVisualization)
{
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
	// Wait for the ROS master to become online
	while (!ros::master::check()) {
		if (m_shouldQuit) {
			quit();
			return;
		}
		msleep(100); // We cannot use ROS rate now because we need the ROS master to come up before
	}

	ros::NodeHandle nh;
	ros::Subscriber m_sub = nh.subscribe(m_topicName.toStdString(),
	                                      1000,
	                                      &ScalarFetcher::fetchScalar, this);

	ros::Rate rate(10); // 10Hz
	while (ros::ok()) {
		if (m_shouldQuit) {
			quit();
			return;
		}

		ros::spinOnce();
		rate.sleep();
	}
}
