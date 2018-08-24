#include "matrixfetcher.h"
#include "matrixvisualization.h"

#include <QDebug>

MatrixFetcher::MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, QObject *parent) :
    DataFetcher(topicName, parent),
    m_matrixVisualization(matrixVisualization),
    m_scalarVisualization(nullptr)
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
	qDebug() << "[MatrixFetcher] run";

	ros::Subscriber m_sub = m_n.subscribe(m_topicName.toStdString(),
	                                      1000,
	                                      &MatrixFetcher::fetchMatrix, this);

	while (ros::ok()) {
		if (m_shouldQuit) {
			qDebug() << "[MatrixFetcher] quitting";
			quit();
			return;
		}

		ros::spinOnce();
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
				m_matrixVisualization->updateGrayscale(scalar->data);
			}
		break;

		default:
			qWarning() << "MatrixFetcher cannot deal with type" << m_visType;
		break;
	}
}
