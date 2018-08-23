#ifndef SCALARFETCHER_H
#define SCALARFETCHER_H

#include "datafetcher.h"
#include "types.h"

#include <QDebug>
#include <QBarSet>
#include <QSplineSeries>

#include <ros/ros.h>
#include "std_msgs/Float64.h"

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization;

class ScalarFetcher : public DataFetcher
{
	Q_OBJECT
public:
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, QObject *parent = nullptr);
	explicit ScalarFetcher(const QString &topicName, ScalarVisualization *scalarVisualization, VisualizationType type, QObject *parent = nullptr);

	void setVisType(VisualizationType type) override;

protected:
	void run() override;

	void fetchScalar(const std_msgs::Float64::ConstPtr& scalar);

private:
	ScalarVisualization *m_scalarVisualization;
};

#endif // SCALARFETCHER_H
