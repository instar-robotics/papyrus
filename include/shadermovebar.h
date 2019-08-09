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
class ThreadShader;

/**
 * @brief The ShaderMoveBar class display a black rectangle as a bar linked to a ShaderProxy to move it
 * on a QGraphicsScene. It is set as parent of the ShaderProxy so the two can be moved together. The black
 * rectangle also contains the name of the box that is displayed by the ShaderProxy.
 */

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

	qreal proxyWidth() const;

	qreal proxyHeight() const;

	void setProxyHeight(qreal proxyHeight);

	void setProxyWidth(qreal proxyWidth);

	QVariant itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value);

	ThreadShader *thread() const;

	void setThread(ThreadShader *thread);

private:
	ShaderProxy *m_proxy;
	ThreadShader *m_thread;
	QString m_title;
	int m_fontSize = 12;
	LinkVisuToBox *m_linkVisuToBox;
	qreal m_proxyWidth;
	qreal m_proxyHeight;
};

#endif // SHADERMOVEBAR_H
