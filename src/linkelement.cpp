#include "linkelement.h"

#include <QPen>
#include <QPainter>

LinkElement::LinkElement(QGraphicsItem *parent)
    : QGraphicsLineItem(parent),
      m_color(defaultLinkColor),
      m_width(defaultLinkWidth)
{
}

/**
 * @brief LinkElement::paint draw the line, with the given color and width
 * @param painter
 * @param option
 * @param widget
 */
void LinkElement::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(option);
	Q_UNUSED(widget);

	QPen pen(m_color);
	pen.setWidthF(m_width);

	if (m_isSecondary)
		pen.setStyle(Qt::DashLine);

	painter->setPen(pen);
	painter->drawLine(line());
}

QColor LinkElement::color() const
{
	return m_color;
}

void LinkElement::setColor(const QColor &color)
{
	m_color = color;
}

qreal LinkElement::width() const
{
	return m_width;
}

void LinkElement::setWidth(const qreal &width)
{
	m_width = width;
}

bool LinkElement::isSecondary() const
{
	return m_isSecondary;
}

void LinkElement::setIsSecondary(bool isSecondary)
{
	m_isSecondary = isSecondary;
}
