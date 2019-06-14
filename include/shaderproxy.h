#ifndef SHADERPROXY_H
#define SHADERPROXY_H

#include <QGraphicsRectItem>
#include <QPen>
#include <QBrush>
#include "shaderwidget.h"
#include "activityfetcher.h"
#include "types.h"
#include "shadermovebar.h"
#include "diagrambox.h"

class ShaderProxy : public QGraphicsProxyWidget
{
	Q_OBJECT

public:
	ShaderProxy(ShaderWidget *widget, ShaderMoveBar *moveBar, DiagramBox *box);
	~ShaderProxy();

	ShaderMoveBar *moveBar() const;

	void setActivityFetcher(ActivityFetcher *activityFetcher);
	void wheelEvent(QGraphicsSceneWheelEvent *event) override;
	void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

signals:
	void proxyDestroyed();

public slots:
	void updateProxy();
	void updateValues(QVector<qreal>* values);

private:
	ShaderWidget *m_widget;
	ShaderMoveBar *m_moveBar;
	ActivityFetcher *m_activityFetcher;
	DiagramBox *m_box;

	int m_rows;
	int m_columns;

	QPoint m_lastPos;
	qreal m_resizeMargin = 7;
	qreal m_moveBarHeight = 20;
};

#endif // SHADERPROXY_H
