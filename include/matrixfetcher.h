#ifndef MATRIXFETCHER_H
#define MATRIXFETCHER_H

#include "scalarvisualization.h"

#include <QString>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

class ScalarVisualization;
class MatrixVisualization;

class MatrixFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, MatrixVisualization *matrixVisualization, VisualizationType type, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& scalar);

private:
	ScalarVisualization *m_scalarVisualization;
	MatrixVisualization *m_matrixVisualization;
	QList<double> m_dataList;
};

#endif // MATRIXFETCHER_H
