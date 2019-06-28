#ifndef SCOPEITEM_H
#define SCOPEITEM_H

#include <QGraphicsItem>
#include <QRectF>
#include <QLineF>

class ScopeItem : public QGraphicsItem
{
public:
	ScopeItem(QGraphicsItem *parent = nullptr);

	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void setMaxRectWidth(qreal width);
	void setMeansRectWidth(qreal width);
	void setMinRectWidth(qreal width);
	void setCurrentRectWidth(qreal width);

	static qreal m_h1;      // Height of the maxRect
	static qreal m_h2;      // Height of the currentRect
	static qreal m_h3;      // Height of the minRect
	static qreal m_hAvg;    // Height of the meansRect
	static qreal barHeight; // Total height of the item

private:
	QRectF m_meansRect;   // Rectangle to draw the means duration
	QRectF m_maxRect;     // Rectangle to draw max duration
	QRectF m_minRect;     // Rectangle to draw min duration
	QRectF m_currentRect; // Line to show current duration
	QLineF m_startLine;   // The vertical line showing the start of the item

};

#endif // SCOPEITEM_H
