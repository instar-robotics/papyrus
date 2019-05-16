#include "openglproxy.h"

void OpenGLProxy::connectProxy(OpenGLWidget *widget)
{
	setWidget(widget);
	connect(widget, SIGNAL(repaint()), this, SLOT(updateProxy()));
}

void OpenGLProxy::updateProxy()
{
	update();
}
