#ifndef OPENGLPROXY_H
#define OPENGLPROXY_H

#include <QGraphicsProxyWidget>
#include <QGraphicsRectItem>
#include "openglwidget.h"
#include "activityfetcher.h"

class OpenGLProxy : public QGraphicsProxyWidget
{
	Q_OBJECT

public:
	OpenGLProxy(OpenGLWidget *widget, QGraphicsRectItem *moveBar);
	~OpenGLProxy();

	QGraphicsRectItem *moveBar() const;

	void setActivityFetcher(ActivityFetcher *activityFetcher);

signals:
	void proxyDestroyed();

public slots:
	void updateProxy();
	void updateValues(QVector<qreal>* values);

private:
	OpenGLWidget *m_widget;
	QGraphicsRectItem *m_moveBar;
	ActivityFetcher *m_activityFetcher;

	int m_rows;
	int m_columns;
};

#endif // OPENGLPROXY_H
