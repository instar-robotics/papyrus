#include "openglbox.h"

OpenGLBox::OpenGLBox(QGraphicsScene *scene, OpenGLWidget *widget):m_widget(widget)
{
	m_proxy = scene->addWidget(m_widget);
	connect(widget, SIGNAL(repaint()), this, SLOT(updateProxy()));
}

void OpenGLBox::updateProxy()
{
	qDebug() << "signal received";
	m_proxy->update();
}
