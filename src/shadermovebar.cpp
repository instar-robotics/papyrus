#include "shadermovebar.h"

#include <QDebug>

ShaderMoveBar::ShaderMoveBar()
{
}

ShaderMoveBar::~ShaderMoveBar()
{
	m_proxy = nullptr;
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

