#include "openglproxy.h"

OpenGLProxy::OpenGLProxy(OpenGLWidget *widget, QGraphicsRectItem *moveBar):
    m_widget(widget),
    m_moveBar(moveBar)
{

	setWidget(m_widget);
	//m_widget->resizeGL(200,200);
	m_moveBar->setRect(0, 0, m_widget->width(), 20);
	m_moveBar->setPen(QPen(Qt::black));
	m_moveBar->setBrush(QBrush(Qt::black));
	setPos(0, m_moveBar->rect().height());
	setParentItem(m_moveBar);

	m_moveBar->setFlag(QGraphicsItem::ItemIsMovable, true);
	m_moveBar->setFlag(QGraphicsItem::ItemIsSelectable, true);

	connect(m_widget, SIGNAL(repaint()), this, SLOT(updateProxy()));
}
OpenGLProxy::~OpenGLProxy()
{
	setParentItem(nullptr);
	delete m_moveBar;
	setWidget(nullptr);
	delete m_widget;
}

void OpenGLProxy::updateProxy()
{
	update();
}

void OpenGLProxy::updateValues(QVector<qreal>* values)
{
	m_widget->updateValues(values);
	//delete values;
}
void OpenGLProxy::setActivityFetcher(ActivityFetcher *activityFetcher)
{
	m_activityFetcher = activityFetcher;
}

QGraphicsRectItem *OpenGLProxy::moveBar() const
{
	return m_moveBar;
}
