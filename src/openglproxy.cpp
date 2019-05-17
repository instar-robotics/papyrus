#include "openglproxy.h"

OpenGLProxy::OpenGLProxy(OpenGLWidget *widget):m_widget(widget)
{
	setWidget(m_widget);
	connect(m_widget, SIGNAL(repaint()), this, SLOT(updateProxy()));
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
	setFlag(QGraphicsItem::ItemIsFocusable);
}
OpenGLProxy::~OpenGLProxy()
{
	emit proxyDestroyed();
}

void OpenGLProxy::updateProxy()
{
	update();
}
