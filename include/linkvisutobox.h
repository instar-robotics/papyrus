#ifndef LINKVISUTOBOX_H
#define LINKVISUTOBOX_H

#include <QGraphicsLineItem>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QDebug>

class LinkVisuToBox : public QGraphicsLineItem
{
public:
	LinkVisuToBox(float xVisu, float yVisu, float xDiagramBox, float yDiagramBox);
	~LinkVisuToBox();

	void centerDiagramBoxMoved(float x, float y);
	void centerVisuMoved(float x, float y);

private:
	float m_xDiagramBox;
	float m_yDiagramBox;
	float m_xVisu;
	float m_yVisu;
	QBrush m_brush;
	QPen m_pen;
	int m_fontSize = 1;
	QColor m_color = Qt::gray;
};

#endif // LINKVISUTOBOX_H
