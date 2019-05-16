#ifndef OPENGLPROXY_H
#define OPENGLPROXY_H

#include <QGraphicsProxyWidget>
#include "openglwidget.h"

class OpenGLProxy : public QGraphicsProxyWidget
{
	Q_OBJECT
public:
	void connectProxy(OpenGLWidget *widget);

signals:
	void proxyDeleted();

public slots:
	void updateProxy();
};

#endif // OPENGLPROXY_H
