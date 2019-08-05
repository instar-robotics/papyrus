#ifndef SHADERMOVEBAR_H
#define SHADERMOVEBAR_H

#include <QGraphicsRectItem>
#include <QPainter>
#include <QBrush>
#include <QFont>
#include <QPointF>
#include "linkvisutobox.h"
#include <QGraphicsSceneMouseEvent>

class ShaderProxy;

class ShaderMoveBar : public QGraphicsRectItem
{
public:
	ShaderMoveBar();
	~ShaderMoveBar();

	ShaderProxy *proxy() const;
	void setProxy(ShaderProxy *proxy);

	void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget) override;

	void setTitle(const QString &title);

	void setLinkVisuToBox(LinkVisuToBox *linkVisuToBox);

	float proxyWidth() const;

	float proxyHeight() const;

	void setProxyHeight(float proxyHeight);

	void setProxyWidth(float proxyWidth);

	QVariant itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value);

private:
	ShaderProxy *m_proxy;
	QString m_title;
	int m_fontSize = 12;
	LinkVisuToBox *m_linkVisuToBox;
	float m_proxyWidth;
	float m_proxyHeight;
};

#endif // SHADERMOVEBAR_H
