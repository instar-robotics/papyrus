#ifndef LINKVISUTOBOX_H
#define LINKVISUTOBOX_H

#include <QGraphicsLineItem>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QDebug>

/**
 * @brief The LinkVisuToBox class draw a gray line to visualize the link between a visualization and its
 * diagram box.
 */

class LinkVisuToBox : public QGraphicsLineItem
{
public:
	LinkVisuToBox(qreal xVisu, qreal yVisu, qreal xDiagramBox, qreal yDiagramBox);
	~LinkVisuToBox();

	void centerDiagramBoxMoved(qreal x, qreal y);
	void centerVisuMoved(qreal x, qreal y);

private:
	qreal m_xDiagramBox;
	qreal m_yDiagramBox;
	qreal m_xVisu;
	qreal m_yVisu;
	QBrush m_brush;
	QPen m_pen;
	int m_fontSize = 1;
	QColor m_color = Qt::gray;
};

#endif // LINKVISUTOBOX_H
