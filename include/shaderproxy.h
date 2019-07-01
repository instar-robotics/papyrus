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
#include <QPainter>
#include <QBrush>

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
	void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget) override;

	qreal moveBarHeight() const;

	ShaderWidget *widget() const;

	void positionWidget(qreal x, qreal y);
	void resizeWidget(int width, int height);

signals:
	void proxyDestroyed();

public slots:
	void updateProxy();
	void updateValues(QVector<qreal>* values);
	void hideDisplay();
	void showDisplay();

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

	// Variables for resizing
	QPoint m_clickPos;
	int m_oldWidth;
	int m_oldHeight;

	// HUD
	int m_margin = 5; //Left margin of scales and down margin for down scales
	int m_marginTop = 10; //Margin at top of Y scale
	int m_marginFont = 2; //Margin between scale and value
	int m_measureSize = 7; //Size of a scale's branch
	int m_measureGap = 13; //Distance between two branches in a scale
	int m_fontSize = 6;
};

#endif // SHADERPROXY_H
