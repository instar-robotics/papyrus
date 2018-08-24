#ifndef MATRIXFETCHER_H
#define MATRIXFETCHER_H

#include "scalarvisualization.h"

#include <QString>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

class ScalarVisualization;

class MatrixFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit MatrixFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;
	void fetchMatrix(const std_msgs::Float64MultiArray::ConstPtr& scalar);

private:
	ScalarVisualization *m_scalarVisualization;
};

#endif // MATRIXFETCHER_H
