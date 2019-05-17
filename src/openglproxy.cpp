#include "openglproxy.h"

OpenGLProxy::OpenGLProxy(OpenGLWidget *widget, QGraphicsRectItem *moveBar):m_widget(widget), m_moveBar(moveBar)
{
	setWidget(m_widget);

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
	emit proxyDestroyed();
	setParentItem(nullptr);
	delete m_moveBar;
}

void OpenGLProxy::updateProxy()
{
	update();
}

QGraphicsRectItem *OpenGLProxy::moveBar() const
{
	return m_moveBar;
}
