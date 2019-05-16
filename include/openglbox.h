#ifndef OPENGLBOX_H
#define OPENGLBOX_H

#include "openglwidget.h"
#include <QGraphicsScene>
#include <QGraphicsProxyWidget>
#include <QObject>
#include <diagramscene.h>

class OpenGLBox : public QObject
{
	Q_OBJECT
public:
	OpenGLBox(QGraphicsScene *scene, OpenGLWidget *widget);

public slots:
	void updateProxy();

private:
	OpenGLWidget * m_widget;
	QGraphicsProxyWidget * m_proxy;
};

#endif // OPENGLBOX_H
