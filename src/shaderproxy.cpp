#include "shaderproxy.h"

ShaderProxy::ShaderProxy(ShaderWidget *widget, ShaderMoveBar *moveBar, DiagramBox * box):
    m_widget(widget),
    m_moveBar(moveBar),
    m_box(box)
{
	setWidget(m_widget);
	m_moveBar->setRect(0,0,m_widget->width(),m_moveBarHeight);
	m_moveBar->setPen(QPen(Qt::black));
	m_moveBar->setBrush(QBrush(Qt::black));
	setParentItem(m_moveBar);
	positionWidget(0, 0);

	m_moveBar->setFlag(QGraphicsItem::ItemIsMovable, true);
	m_moveBar->setFlag(QGraphicsItem::ItemIsSelectable, true);

	connect(m_widget, SIGNAL(repaint()), this, SLOT(updateProxy()));
}

ShaderProxy::~ShaderProxy()
{
	if (m_activityFetcher != nullptr) {
		m_activityFetcher->setShouldQuit(true);
		m_activityFetcher->wait(1000);
		delete m_activityFetcher;
	}
	if (m_box != nullptr) {
		m_box->setDisplayedProxy(nullptr);
		m_box->setActivityVisualizer(nullptr);
		m_box->setIsActivityVisuEnabled(false);
	}

	setParentItem(nullptr);
	delete m_moveBar;
}

void ShaderProxy::updateProxy()
{
	update();
}

void ShaderProxy::updateValues(QVector<qreal>* values)
{
	m_widget->updateValues(values);
	delete values;
}

void ShaderProxy::hideDisplay()
{
	hide();
}

void ShaderProxy::showDisplay()
{
	show();
}

ShaderWidget *ShaderProxy::widget() const
{
	return m_widget;
}

void ShaderProxy::positionWidget(qreal x, qreal y)
{
	m_moveBar->setRect(x, y-m_moveBarHeight, m_widget->width(), m_moveBarHeight);
	setPos(x, y);
}

void ShaderProxy::resizeWidget(int width, int height)
{
	resize(width, height);
	m_moveBar->setRect(pos().x(), pos().y()-m_moveBarHeight, width, m_moveBarHeight);

	//Resize the widget if it is too small
	if(width<m_widget->minWidth())
	{
		m_widget->resize(m_widget->minWidth(), m_widget->height());
		m_moveBar->setRect(pos().x(), pos().y()-m_moveBarHeight, m_widget->minWidth(),m_moveBarHeight);
	}
	if(height<m_widget->minHeight())
		m_widget->resize(m_widget->width(), m_widget->minHeight());
}

qreal ShaderProxy::moveBarHeight() const
{
	return m_moveBarHeight;
}
void ShaderProxy::setActivityFetcher(ActivityFetcher *activityFetcher)
{
	m_activityFetcher = activityFetcher;
}

ShaderMoveBar *ShaderProxy::moveBar() const
{
	return m_moveBar;
}

//Override the diagram scene's wheel event to focus it on the zoom in/zoom out in opengl
void ShaderProxy::wheelEvent(QGraphicsSceneWheelEvent *event)
{
	if (event->modifiers() & Qt::ShiftModifier) {
		if(event->delta() > 0)
			m_widget->scalePlanes()->updateScale(m_widget->scalePlanes()->max()*1.2);
		else
			m_widget->scalePlanes()->updateScale(m_widget->scalePlanes()->max()/1.2);
	}
	else
		m_widget->wheelTurned(event->delta());
}
void ShaderProxy::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
	qreal mouseX = event->pos().x();
	qreal mouseY = event->pos().y();

	//Change the cursor if the cursor is on a widget's border to show that it is resizable
	if (mouseX >= m_widget->width() - m_resizeMargin && mouseY >= m_widget->height() - m_resizeMargin)
		setCursor(QCursor(Qt::SizeFDiagCursor));
	else if (mouseX >= m_widget->width() - m_resizeMargin)
		setCursor(QCursor(Qt::SizeHorCursor));
	else if (mouseY >= m_widget->height() - m_resizeMargin)
		setCursor(QCursor(Qt::SizeVerCursor));
	else
		setCursor(QCursor(Qt::ArrowCursor));
}

void ShaderProxy::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	m_lastPos = QPoint(event->pos().x(), event->pos().y());
	m_widget->mousePressed(m_lastPos);

	//Set variables used for resizing
	m_clickPos = QPoint(event->pos().x(), event->pos().y());
	m_oldWidth = m_widget->width();
	m_oldHeight = m_widget->height();
}
void ShaderProxy::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	QPoint pos(event->pos().x(), event->pos().y());
	if (event->buttons() & Qt::LeftButton)
	{
		int dx = pos.x() - m_lastPos.x();
		int dy = pos.y() - m_lastPos.y();
		int width = m_widget->width();
		int height = m_widget->height();

		//Resize the widget if the the user clicked on a widget's border
		if (m_clickPos.x() >= m_oldWidth - m_resizeMargin && m_clickPos.y() >= m_oldHeight - m_resizeMargin)
		{
			resizeWidget(width+dx, height+dy);
		}
		else if (m_clickPos.x() >= m_oldWidth - m_resizeMargin)
		{
			resizeWidget(width+dx, height);
		}
		else if (m_clickPos.y() >= m_oldHeight - m_resizeMargin)
			resizeWidget(width, height+dy);

		//If the user doesn't resize the widget, transmit the mouseMoveEvent the opengl
		else
			m_widget->mouseMoved(pos, LEFT_BUTTON);
	}
	if (event->buttons() & Qt::RightButton)
	{
		if(event->modifiers() & Qt::ShiftModifier)
		{
			m_widget->mouseMoved(pos, RIGHT_SHIFT_BUTTON);
		}
		else
		{
			m_widget->mouseMoved(pos, RIGHT_BUTTON);
		}
	}
	m_lastPos = pos;
}

void ShaderProxy::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	QGraphicsProxyWidget::paint(painter, option, widget);

	// HUD
	QBrush brush(Qt::black, Qt::SolidPattern);
	painter->setPen(Qt::NoPen); //Remove the pen to fill the polygon
	painter->setBrush(brush);

	QSize widgetSize = m_widget->size();
	QVector<QPoint> points;
	points.push_back(QPoint(widgetSize.width(),widgetSize.height()));
	points.push_back(QPoint(widgetSize.width()-18,widgetSize.height()));
	points.push_back(QPoint(widgetSize.width(),widgetSize.height()-18));

	painter->drawPolygon(points);

	if(m_widget->scalePlanes() != nullptr && m_widget->width() >= m_widget->startWidth())
	{
		painter->setPen(QPen(brush, 1)); //Set the pen for scales

		//Scale measures for each axe
		QVector<QLine> lines;
		QFont font = painter->font();
		font.setPixelSize(m_fontSize);
		painter->setFont(font);
		float max = m_widget->scalePlanes()->max();
		float rows = m_widget->scalePlanes()->rows();
		float columns = m_widget->scalePlanes()->columns();
		float gap = 0.0;

		/*Y axe*/
		//Lines
		lines.push_back(QLine(QPoint(m_margin,m_marginTop), QPoint(m_margin, m_marginTop + (m_widget->nbMeasuresY()-1)*m_measureGap)));
		for(int i = 0; i<m_widget->nbMeasuresY(); i++)
		{
			lines.push_back(QLine(QPoint(m_margin, m_marginTop + i*m_measureGap), QPoint(m_margin + m_measureSize, m_marginTop + i*m_measureGap)));
		}
		//Values
		gap = 2*max/(m_widget->nbMeasuresY()-1);
		for(int i = 0; i<m_widget->nbMeasuresY(); i++)
		{
			float value = max - gap*i;
			QString str = QString::fromStdString(std::to_string(value));
			str.resize(4);
			painter->drawText(m_margin + m_measureSize + m_marginFont, m_marginTop + i*m_measureGap + m_fontSize/2, str);
		}
		painter->drawText(m_margin * 2 - (m_fontSize+1)/2, m_marginTop + (m_widget->nbMeasuresY()-1)*m_measureGap + m_margin + (m_fontSize+1)/2, "Z");
		// The Y and Z axes for 3d displayed are inversed in the matricial space

		/*X axe*/
		//Lines
		lines.push_back(QLine(QPoint(m_marginTop,widgetSize.height()-m_margin), QPoint((m_widget->nbMeasuresXZ()-1) * m_measureGap-1 + m_marginTop,widgetSize.height()-m_margin)));
		for(int i = 0; i<m_widget->nbMeasuresXZ(); i++)
		{
			lines.push_back(QLine(QPoint(i*m_measureGap+m_marginTop,widgetSize.height()-m_margin), QPoint(i*m_measureGap+m_marginTop,widgetSize.height()-m_margin-m_measureSize)));
		}
		//Values
		gap = columns/(m_widget->nbMeasuresXZ()-1);
		for(int i = 0; i<m_widget->nbMeasuresXZ(); i++)
		{
			int value = gap*i;
			QString str = QString::fromStdString(std::to_string(value));
			painter->drawText(i*m_measureGap+m_marginTop - (m_fontSize+1)/2, widgetSize.height()-m_margin-m_measureSize-m_marginFont, str);
		}
		painter->drawText(m_margin - (m_fontSize+1)/2, widgetSize.height()-m_margin-m_marginFont, "X");

		/*Z axe*/
		//Lines
		lines.push_back(QLine(QPoint(m_widget->nbMeasuresXZ() * m_measureGap + m_margin + m_marginTop,widgetSize.height()-m_margin), QPoint(QPoint(m_widget->nbMeasuresXZ() * m_measureGap + m_margin + m_marginTop + (m_widget->nbMeasuresXZ()-1) * m_measureGap,widgetSize.height()-m_margin))));
		for(int i = 0; i<m_widget->nbMeasuresXZ(); i++)
		{
			lines.push_back(QLine(QPoint(m_widget->nbMeasuresXZ() * m_measureGap + m_margin + m_marginTop + i*m_measureGap,widgetSize.height()-m_margin), QPoint(m_widget->nbMeasuresXZ() * m_measureGap + m_margin + m_marginTop + (i*m_measureGap),widgetSize.height()-m_margin-m_measureSize)));
		}
		//Values
		gap = rows/(m_widget->nbMeasuresXZ()-1);
		for(int i = 0; i<m_widget->nbMeasuresXZ(); i++)
		{
			int value = gap*i;
			QString str = QString::fromStdString(std::to_string(value));
			painter->drawText(m_widget->nbMeasuresXZ()*m_measureGap+m_margin+m_marginTop+i*m_measureGap-(m_fontSize+1)/2, widgetSize.height()-m_margin-m_measureSize-m_marginFont, str);
		}
		painter->drawText(m_widget->nbMeasuresXZ() * m_measureGap + 2*m_margin - (m_fontSize+1)/2, widgetSize.height()-m_margin-m_marginFont, "Y");
		// The Y and Z axes for 3d displayed are inversed in the matricial space

		painter->drawLines(lines);
	}

}
