#ifndef SHADERWIDGET_H
#define SHADERWIDGET_H

#include <QDebug>
#include <QObject>
#include <QWidget>
#include <QTimer>
#include <QTime>
#include <QWheelEvent>
#include <QMouseEvent>
#include <sstream>
#include <QGraphicsProxyWidget>

#include <QOpenGLWidget>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>

#include <QVector>
#include <QVector3D>

#include <QPainter>
#include <QBrush>

#include "mathtransfo.h"
#include "camera.h"
#include "light.h"
#include "types.h"
#include "shaderscaleplanes.h"
#include "shaderaxesplanes.h"

enum class Attribute { Vertex, Normal, Color };

class ShaderWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	ShaderWidget();
	~ShaderWidget();

	void initializeGL();
	virtual void updateValues(QVector<qreal>* values) = 0;

	// Events
	void wheelTurned(int delta);
	void mousePressed(QPoint pos);
	void mouseMoved(QPoint pos, MouseControl mouseControl);

	int minWidth() const;
	int minHeight() const;

signals:
	void repaint();

protected:
	//OpenGL
	void paintGL() override;
	void resizeGL(int width, int height) override;
	void initGPUbuffers();
	void destroyGPUbuffers();
	void initShaders();
	virtual void addShaders() = 0;
	virtual void initVectors() = 0;
	virtual void fillVectors() = 0;
	void clearVectors();
	void updateScene();

	QColor calculateColor(float const& value, const float &max_value);

private:
	// FPS
	QTime m_timer;
	size_t m_frame_count{};
	size_t m_last_count{};

protected:

	//Vectors
	QVector<GLuint> m_indexes; //save the index of vertexes to use in the drawing order
	QVector<QVector3D> m_vertexes; //save the coordinates of all the vertexes for the render
	QVector<QVector3D> m_normals; //save the vertexes' normals for light gesture
	QVector<QVector3D> m_colors; //save the vertexes' colors for the render

	// GPU Buffer
	QOpenGLBuffer m_indexbuffer{QOpenGLBuffer::IndexBuffer};
	QOpenGLBuffer m_vertexbuffer{QOpenGLBuffer::VertexBuffer};
	QOpenGLBuffer m_normalbuffer{QOpenGLBuffer::VertexBuffer};
	QOpenGLBuffer m_colorbuffer{QOpenGLBuffer::VertexBuffer};

	// Shader program
	QOpenGLShaderProgram m_program;

	// Scale planes
	ShaderScalePlanes *m_scalePlanes = nullptr;

	// 3D scene
	float m_gap = 0.2f;
	Camera m_camera;
	Light m_light;
	float m_range = 4.0f;

	// widget size
	int m_startWidth = 300;
	int m_startHeight = 300;
	int m_minWidth = 150;
	int m_minHeight = 150;

	GLenum drawingType = GL_TRIANGLES;

};

#endif // SHADERWIDGET_H
