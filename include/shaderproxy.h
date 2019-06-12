#ifndef SHADERPROXY_H
#define SHADERPROXY_H

#include <QGraphicsProxyWidget>
#include <QGraphicsRectItem>
#include <QPen>
#include <QBrush>
#include "shaderwidget.h"
#include "activityfetcher.h"
#include "types.h"

class ShaderProxy : public QGraphicsProxyWidget
{
	Q_OBJECT

public:
	ShaderProxy(ShaderWidget *widget, QGraphicsRectItem *moveBar);
	~ShaderProxy();

	QGraphicsRectItem *moveBar() const;

	void setActivityFetcher(ActivityFetcher *activityFetcher);
	void wheelEvent(QGraphicsSceneWheelEvent *event) override;

signals:
	void proxyDestroyed();

public slots:
	void updateProxy();
	void updateValues(QVector<qreal>* values);

private:
	ShaderWidget *m_widget;
	QGraphicsRectItem *m_moveBar;
	ActivityFetcher *m_activityFetcher;

	int m_rows;
	int m_columns;
};

#endif // SHADERPROXY_H
