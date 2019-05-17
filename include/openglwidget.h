#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QDebug>
#include <QObject>
#include <QWidget>
#include <QtGui>
#include <QOpenGLWidget>
#include <QTimer>
#include <QTime>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QPoint>
#include <sstream>
#include <QGraphicsProxyWidget>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "mathtransfo.h"

#define MAP_SIZE 5.0

class OpenGLWidget : public QOpenGLWidget
{
	Q_OBJECT

public:

	//OpenGLWidget();
	//~OpenGLWidget();

	void initializeGL(int const& distance = -10,
	                  int const& xRot = 450,
	                  int const& yRot = 0,
	                  int const& zRot = 0,
	                  int const& xTran = 0,
	                  int const& yTran = 0,
	                  int const& zTran = 0,
	                  int const& rot_speed = 10
	        );
	virtual void paintGL();
	void resizeGL(int width, int height);

	void start3dCameraView();
	void end3dCameraView();
	void startHUDCameraView();
	void endHUDCameraView();

	virtual void paint3dObjects();
	virtual void paintHUD();
	virtual void calculation();

	void calculateFPS();
	void displayMousePos() const;

	void wheelEvent(QWheelEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void keyPressEvent(QKeyEvent* event);
	void keyReleaseEvent(QKeyEvent* event);

	void rotateView(int x, int y, int z);
	void translateView(int x, int y, int z);
	void reinitializeView();

	bool mousePosDisplayed() const;
	void setMousePosDisplayed(bool mousePosDisplayed);
	bool fpsDisplayed() const;
	void setFpsDisplayed(bool fpsDisplayed);

	QColor calculateColor(float const& value, const float &max_value, const float &min_value);

signals:
	void repaint();

protected:

	bool m_mousePosDisplayed = false;
	bool m_fpsDisplayed = false;
	float m_distance;
	int m_xRot;
	int m_yRot;
	int m_zRot;
	float m_xTran;
	float m_yTran;
	float m_zTran;
	float m_xPos;
	float m_zPos;
	int m_rot_speed;
	QVector<QVector3D>  m_vertices;
	int m_vertices_by_x;
	int m_vertices_by_z;
	int m_quads_by_x;
	int m_quads_by_z;
	QTimer m_timer;
	QTime m_last_time;
	int m_last_frame_count;
	int m_current_frame_count;
	QPoint m_last_pos;
	QGraphicsProxyWidget *m_proxy;
	QKeyEvent* m_keyPressed;
};

#endif // OPENGLWIDGET_H
