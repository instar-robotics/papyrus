#ifndef DATAFETCHER_H
#define DATAFETCHER_H

#include "types.h"

#include <QThread>
#include <QString>
#include <QList>
#include <ros/ros.h>

//QT_CHARTS_USE_NAMESPACE

class DataFetcher : public QThread
{
	Q_OBJECT

public:
	explicit DataFetcher(const QString &topicName, QObject *parent = nullptr);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	VisualizationType visType() const;
	virtual void setVisType(VisualizationType type) = 0;

protected:
	QString m_topicName;
	bool m_shouldQuit;
//	ros::NodeHandle m_n;
	VisualizationType m_visType;

signals:
	void newMatrix(QList<double> *values);
};

#endif // DATAFETCHER_H
