#ifndef LINKELEMENT_H
#define LINKELEMENT_H

#include <QGraphicsLineItem>
#include <QColor>

/**
 * @brief The LinkElement class represent one line segment of a @Link. A standard @Link has just one
 * @LinkElement (the straight line) while a self-looping @Link has 3 @LinkElement s.
 * This was subclassed so that we could change its width, color, opacity, etc.
 */

const QColor defaultLinkColor = QColor(48, 140, 198);
const qreal defaultLinkWidth = 1.3;

class LinkElement : public QGraphicsLineItem
{
public:
	explicit LinkElement(QGraphicsItem *parent = nullptr);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	QColor color() const;
	void setColor(const QColor &color);

	qreal width() const;
	void setWidth(const qreal &width);

	bool isSecondary() const;
	void setIsSecondary(bool isSecondary);

private:
	QColor m_color;     // the color in which to paint this element
	qreal m_width;      // the line width with which to paint this element
	bool m_isSecondary; // whether this element is part of a secondary Link
};

#endif // LINKELEMENT_H
