#ifndef OPENGLPROXY_H
#define OPENGLPROXY_H

#include <QGraphicsProxyWidget>
#include <QGraphicsRectItem>
#include "openglwidget.h"

class OpenGLProxy : public QGraphicsProxyWidget
{
	Q_OBJECT
public:
	OpenGLProxy(OpenGLWidget *widget, QGraphicsRectItem *moveBar);
	~OpenGLProxy();

	QGraphicsRectItem *moveBar() const;

signals:
	void proxyDestroyed();

public slots:
	void updateProxy();

private:
	OpenGLWidget *m_widget;
	QGraphicsRectItem *m_moveBar;
};

#endif // OPENGLPROXY_H
