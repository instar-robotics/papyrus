#include "shadermovebar.h"

#include <QDebug>

ShaderMoveBar::ShaderMoveBar():
    m_proxy(nullptr),
    m_thread(nullptr),
    m_linkVisuToBox(nullptr)
{
	setFlag(ItemSendsGeometryChanges, ItemIsMovable);
}

ShaderMoveBar::~ShaderMoveBar()
{
	m_proxy = nullptr;
	m_thread = nullptr;
	m_linkVisuToBox = nullptr;
}

ShaderProxy* ShaderMoveBar::proxy() const
{
	return m_proxy;
}

void ShaderMoveBar::setProxy(ShaderProxy *proxy)
{
	m_proxy = proxy;
}

void ShaderMoveBar::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	QGraphicsRectItem::paint(painter, option, widget);

	// Draw the title of the box or its type

	QBrush brush(Qt::white, Qt::SolidPattern);
	painter->setBrush(brush);
	painter->setPen(QPen(brush, 1));

	QFont font = painter->font();
	font.setPixelSize(m_fontSize);
	painter->setFont(font);

	painter->drawText(rect(), Qt::AlignCenter, m_title);

}

void ShaderMoveBar::setTitle(const QString &title)
{
	m_title = title;
}

QVariant ShaderMoveBar::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
	if ((change == QGraphicsItem::ItemPositionChange || change == QGraphicsItem::ItemScenePositionHasChanged) && scene()) {
		// Get coordinate of the target new position
		QPointF newPos = value.toPointF();
		if(m_linkVisuToBox != nullptr)
			m_linkVisuToBox->centerVisuMoved(newPos.x()+m_proxyWidth/2, newPos.y()+m_proxyHeight/2);
		return newPos;
	}
	return QGraphicsItem::itemChange(change, value);
}

void ShaderMoveBar::setLinkVisuToBox(LinkVisuToBox *linkVisuToBox)
{
	m_linkVisuToBox = linkVisuToBox;
}

float ShaderMoveBar::proxyWidth() const
{
	return m_proxyWidth;
}

float ShaderMoveBar::proxyHeight() const
{
	return m_proxyHeight;
}

void ShaderMoveBar::setProxyHeight(float proxyHeight)
{
	m_proxyHeight = proxyHeight;
}

void ShaderMoveBar::setProxyWidth(float proxyWidth)
{
	m_proxyWidth = proxyWidth;
}
ThreadShader *ShaderMoveBar::thread() const
{
	return m_thread;
}

void ShaderMoveBar::setThread(ThreadShader *thread)
{
	m_thread = thread;
}
