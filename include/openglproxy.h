#ifndef OPENGLPROXY_H
#define OPENGLPROXY_H

#include <QGraphicsProxyWidget>
#include "openglwidget.h"

class OpenGLProxy : public QGraphicsProxyWidget
{
	Q_OBJECT
public:
	OpenGLProxy(OpenGLWidget *widget);
	~OpenGLProxy();

signals:
	void proxyDestroyed();

public slots:
	void updateProxy();

private:
	OpenGLWidget *m_widget;
};

#endif // OPENGLPROXY_H
