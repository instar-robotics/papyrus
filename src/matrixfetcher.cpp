#include "matrixfetcher.h"
#include "matrixvisualization.h"

#include <QDebug>

MatrixFetcher::MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, QObject *parent) :
    DataFetcher(topicName, parent),
    m_scalarVisualization(nullptr),
    m_matrixVisualization(matrixVisualization)
{
	qDebug() << "[MatrixFetcher] created on topic" << m_topicName << "in matrix mode";

	qDebug() << "[MatrixFetcher] start in matrix mode";
	start();
}

MatrixFetcher::MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, VisualizationType type, QObject *parent) :
    MatrixFetcher(topicName, matrixVisualization, parent)
{
	setVisType(type);
}

MatrixFetcher::MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent) :
    DataFetcher(topicName, parent),
    m_scalarVisualization(scalarVisualization),
    m_matrixVisualization(nullptr)
{
	qDebug() << "[MatrixFetcher] created on topic" << m_topicName << "in scalar mode";

	qDebug() << "[MatrixFetcher] start in scalar mode";
	start();
}

MatrixFetcher::MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent) :
    MatrixFetcher(topicName, scalarVisualization, parent)
{
	setVisType(type);
}

/**
 * @brief MatrixFetcher::setVisType changes the visualization type if it matches the allowed ones
 * for this fetcher
 * @param type
 */
void MatrixFetcher::setVisType(VisualizationType type)
{
	if (type == m_visType)
		return;

	m_visType = type;
}

void MatrixFetcher::run()
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
	                                      &MatrixFetcher::fetchMatrix, this);

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

void MatrixFetcher::fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr &scalar)
{
	switch (m_visType) {
		case BAR:
			if (m_scalarVisualization != nullptr) {
				m_scalarVisualization->updateBarValues(scalar->data);
			}
		break;

		case GRAPH:
			if (m_scalarVisualization != nullptr) {
				m_scalarVisualization->pushGraphValues(scalar->data);
			}
		break;

		case GRAYSCALE:
			if (m_matrixVisualization != nullptr) {
				m_dataList.clear();
				for(unsigned int i = 0; i < scalar->data.size(); i += 1)
					m_dataList.append(scalar->data.at(i));

				emit newMatrix(&m_dataList);
			}
		break;

		default:
			qWarning() << "MatrixFetcher cannot deal with type" << m_visType;
		break;
	}
}
