#include "linkvisutobox.h"

LinkVisuToBox::LinkVisuToBox(float xVisu, float yVisu, float xDiagramBox, float yDiagramBox):
    QGraphicsLineItem(xVisu, yVisu, xDiagramBox, yDiagramBox)
{
	m_xVisu = xVisu;
	m_yVisu = yVisu;
	m_xDiagramBox = xDiagramBox;
	m_yDiagramBox = yDiagramBox;
	m_brush = QBrush(m_color, Qt::SolidPattern);
	m_pen = QPen(m_brush, 1);
	setPen(m_pen);
	setZValue(-1.0);
}
LinkVisuToBox::~LinkVisuToBox()
{
}

void LinkVisuToBox::centerDiagramBoxMoved(float x, float y)
{
	m_xDiagramBox = x;
	m_yDiagramBox = y;

	setLine(QLineF(m_xVisu, m_yVisu, m_xDiagramBox, m_yDiagramBox));
}
void LinkVisuToBox::centerVisuMoved(float x, float y)
{
	qDebug() << m_xVisu << m_yVisu << x << y;
	m_xVisu = x;
	m_yVisu = y;

	setLine(QLineF(m_xVisu, m_yVisu, m_xDiagramBox, m_yDiagramBox));
}
