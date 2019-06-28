#include "scopeitem.h"

#include <QPainter>
#include <QDebug>

qreal ScopeItem::m_h1 = 5;
qreal ScopeItem::m_h2 = 20;
qreal ScopeItem::m_h3 = 5;
qreal ScopeItem::m_hAvg = 2;
qreal ScopeItem::barHeight = ScopeItem::m_h1 + ScopeItem::m_h2 + ScopeItem::m_h3;

ScopeItem::ScopeItem(QGraphicsItem *parent)
    : QGraphicsItem(parent),
      m_meansRect(QPointF(0, m_h1 + m_h2 / 2 - m_hAvg / 2), QPointF(0, m_h1 + m_h2 / 2 + m_hAvg / 2)),
      m_maxRect(QPointF(0, 0), QPointF(0, m_h1)),
      m_minRect(QPointF(0, m_h1 + m_h2), QPointF(0, m_h1 + m_h2 + m_h3)),
      m_currentRect(QPointF(0, m_h1), QPointF(0, m_h1 + m_h2))
{

}

QRectF ScopeItem::boundingRect() const
{
//	qreal w = qMax(m_maxRect.rect().width(), qMax(m_meansRect.rect().width(), m_minRect.rect().width()));
	qreal w = qMax(m_maxRect.width(), qMax(m_meansRect.width(), qMax(m_minRect.width(), m_currentRect.width())));
	qreal h = m_h1 + m_h2 + m_h3; // do not put m_hAvg in here because it's inside

	return QRectF(0, 0, w, h);
}

void ScopeItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(option);
	Q_UNUSED(widget);

	painter->fillRect(m_maxRect, Qt::red);
	painter->fillRect(m_currentRect, QColor(0, 143, 17));
	painter->fillRect(m_minRect, Qt::blue);
	painter->fillRect(m_meansRect, QColor(Qt::gray).lighter());
}

void ScopeItem::setMaxRectWidth(qreal width)
{
	m_maxRect.setWidth(width);
}

void ScopeItem::setMeansRectWidth(qreal width)
{
	m_meansRect.setWidth(width);
}

void ScopeItem::setMinRectWidth(qreal width)
{
	m_minRect.setWidth(width);
}

void ScopeItem::setCurrentRectWidth(qreal width)
{
	m_currentRect.setWidth(width);
}
