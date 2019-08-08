#include "shaderproxy.h"

ShaderProxy::ShaderProxy(ShaderWidget *widget, ShaderMoveBar *moveBar, DiagramBox * box, QMutex *mutex):
    m_widget(widget),
    m_moveBar(moveBar),
    m_box(box),
    m_linkToBox(nullptr),
    m_mutex(mutex)
{
	setWidget(m_widget);
	m_moveBar->setRect(0,0,m_widget->width(),m_moveBarHeight);
	m_moveBar->setPen(QPen(Qt::black));
	m_moveBar->setBrush(QBrush(Qt::black));
	setParentItem(m_moveBar);

	positionWidget(0, 0);

	m_moveBar->setFlag(QGraphicsItem::ItemIsMovable, true);
	m_moveBar->setFlag(QGraphicsItem::ItemIsSelectable, true);
	setFlag(ItemSendsGeometryChanges, ItemIsMovable);

	// Display the box's title or its type on the move bar
	QString title;
	if(box->title().size()>0)
		title = box->title();
	else
		title = box->name();
	m_moveBar->setTitle(title);

	m_moveBar->setProxyWidth(m_widget->width());
	m_moveBar->setProxyHeight(m_widget->height());

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
	if(m_moveBar != nullptr)
		delete m_moveBar;
	if(m_linkToBox != nullptr)
		delete m_linkToBox;
}

void ShaderProxy::updateProxy()
{
	update();
	m_mutex->lock();
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

LinkVisuToBox *ShaderProxy::linkToBox() const
{
	return m_linkToBox;
}

void ShaderProxy::setLinkToBox(LinkVisuToBox *linkToBox)
{
	m_linkToBox = linkToBox;
}

ShaderWidget *ShaderProxy::widget() const
{
	return m_widget;
}

void ShaderProxy::positionWidget(qreal x, qreal y)
{
	m_moveBar->setPos(x, y);
	setPos(0, m_moveBarHeight);
}

void ShaderProxy::resizeWidget(int width, int height)
{

	//Resize the widget if it is too small
	if(width<m_widget->minWidth() && height<m_widget->minHeight())
	{
		m_widget->resize(m_widget->minWidth(), m_widget->minHeight());
		m_moveBar->setRect(0, 0, m_widget->minWidth(),m_moveBarHeight);
	}
	else if(width<m_widget->minWidth())
	{
		m_widget->resize(m_widget->minWidth(), height);
		m_moveBar->setRect(0, 0, m_widget->minWidth(),m_moveBarHeight);
	}
	else if(height<m_widget->minHeight())
	{
		m_widget->resize(width, m_widget->minHeight());
		m_moveBar->setRect(0, 0, width, m_moveBarHeight);
	}
	else
	{
		m_widget->resize(width, height);
		m_moveBar->setRect(0, 0, width, m_moveBarHeight);
	}

	m_moveBar->setProxyWidth(m_widget->width());
	m_moveBar->setProxyHeight(m_widget->height());

	if(m_linkToBox != nullptr)
		m_linkToBox->centerVisuMoved(m_moveBar->scenePos().x()+width/2.0, m_moveBar->scenePos().y()+m_moveBarHeight+height/2.0);
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
		{
			m_widget->updateScale(9.0/8.0); // multiplied by 1.125
			m_max *= (8.0/9.0);
		}
		else
		{
			m_widget->updateScale(8.0/9.0); // divided by 1.125
			m_max *= (9.0/8.0);
		}
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
	if((event->buttons() & Qt::LeftButton) && (event->modifiers() & Qt::ShiftModifier))
		m_widget->mousePressed(m_lastPos, LEFT_SHIFT_BUTTON);
	else
		m_widget->mousePressed(m_lastPos, OTHER);
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
	QVector<QPoint> points; //Drax the triangle at the right down corner that marks the resizable zone
	points.push_back(QPoint(widgetSize.width(),widgetSize.height()));
	points.push_back(QPoint(widgetSize.width()-18,widgetSize.height()));
	points.push_back(QPoint(widgetSize.width(),widgetSize.height()-18));

	painter->drawPolygon(points);

	//Set pen to write text and draw lines
	painter->setPen(QPen(brush, 1));

	ShaderMatrix *matrix = dynamic_cast<ShaderMatrix*>(m_widget);
	ShaderCircular *circular = dynamic_cast<ShaderCircular*>(m_widget);
	ShaderPolar *polar = dynamic_cast<ShaderPolar*>(m_widget);
	ShaderCompass *compass = dynamic_cast<ShaderCompass*>(m_widget);

	if((matrix != nullptr || circular != nullptr || polar != nullptr || compass != nullptr)
	        && m_widget->width() >= m_widget->startWidth())
	{
		QFont font = painter->font();
		font.setPixelSize(m_fontSize);
		painter->setFont(font);

		//Scale measures for each axe
		QVector<QLine> lines;
		float rows = 0.0;
		float columns = 0.0;
		if(matrix != nullptr)
		{
			if(matrix->scalePlanes() != nullptr)
			{
				rows = matrix->scalePlanes()->rows();
				columns = matrix->scalePlanes()->columns();
			}
		}
		qreal gap = 0.0;
		/*Y axe*/
		//Lines
		lines.push_back(QLine(QPoint(m_margin,m_marginTop), QPoint(m_margin, m_marginTop + (m_widget->nbMeasuresY()-1)*m_measureGap)));
		for(int i = 0; i<m_widget->nbMeasuresY(); i++)
		{
			lines.push_back(QLine(QPoint(m_margin, m_marginTop + i*m_measureGap), QPoint(m_margin + m_measureSize, m_marginTop + i*m_measureGap)));
		}
		//Values
		gap = 2*m_max/(m_widget->nbMeasuresY()-1);
		for(int i = 0; i<m_widget->nbMeasuresY(); i++)
		{
			float value = m_max - gap*i;
			QString str = QString::fromStdString(std::to_string(value));
			str.resize(4);
			painter->drawText(m_margin + m_measureSize + m_marginFont, m_marginTop + i*m_measureGap + m_fontSize/2, str);
		}
		painter->drawText(m_margin * 2 - (m_fontSize+1)/2, m_marginTop + (m_widget->nbMeasuresY()-1)*m_measureGap + m_margin + (m_fontSize+1)/2, "Z");
		// The Y and Z axes for 3d displayed are inversed in the matricial space

		if(matrix != nullptr || compass != nullptr)
		{
			/*X axe*/
			//Lines
			lines.push_back(QLine(QPoint(m_marginTop,widgetSize.height()-m_margin), QPoint((m_widget->nbMeasuresXZ()-1) * m_measureGap-1 + m_marginTop,widgetSize.height()-m_margin)));
			for(int i = 0; i<m_widget->nbMeasuresXZ(); i++)
			{
				lines.push_back(QLine(QPoint(i*m_measureGap+m_marginTop,widgetSize.height()-m_margin), QPoint(i*m_measureGap+m_marginTop,widgetSize.height()-m_margin-m_measureSize)));
			}
			//Values
			float value = 0.0;

			if(matrix != nullptr)
				gap = columns/(m_widget->nbMeasuresXZ()-1);
			else
				gap = 2.0*m_max/(m_widget->nbMeasuresXZ()-1);

			for(int i = 0; i<m_widget->nbMeasuresXZ(); i++)
			{
				if(compass != nullptr)
					value = gap*i;
				else
					value = gap*i - m_max;
				QString str = QString::fromStdString(std::to_string(value));
				str.resize(3);
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
			if(matrix != nullptr)
				gap = rows/(m_widget->nbMeasuresXZ()-1);
			else
				gap = 2*m_max/(m_widget->nbMeasuresXZ()-1);

			for(int i = 0; i<m_widget->nbMeasuresXZ(); i++)
			{
				if(matrix != nullptr)
					value = gap*i;
				else
					value = gap*i - m_max;
				QString str = QString::fromStdString(std::to_string(value));
				str.resize(3);
				painter->drawText(m_widget->nbMeasuresXZ()*m_measureGap+m_margin+m_marginTop+i*m_measureGap-(m_fontSize+1)/2, widgetSize.height()-m_margin-m_measureSize-m_marginFont, str);
			}
			painter->drawText(m_widget->nbMeasuresXZ() * m_measureGap + 2*m_margin - (m_fontSize+1)/2, widgetSize.height()-m_margin-m_marginFont, "Y");
			// The Y and Z axes for 3d displayed are inversed in the matricial space
		}
		painter->drawLines(lines);
	}
}
