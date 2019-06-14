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

#include "mathtransfo.h"
#include "camera.h"
#include "light.h"
#include "types.h"

enum class Attribute { Vertex, Normal, Color };

class ShaderWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	ShaderWidget();
	~ShaderWidget();

	void initializeGL();
	virtual void updateValues(QVector<qreal>* values);

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
	virtual void initShaders();
	virtual void initVectors();
	virtual void fillVectors();
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
	QVector<GLuint> m_wireframeIndexes; //save the indexes in the drawing order of the wireframe
	QVector<QVector3D> m_wireframeVertexes; //save vertexes just a little bit higher than original ones
	QVector<QVector3D> m_wireframeColors; //save the vertexes' colors for the wireframe

	// GPU Buffer
	QOpenGLBuffer m_indexbuffer{QOpenGLBuffer::IndexBuffer};
	QOpenGLBuffer m_vertexbuffer{QOpenGLBuffer::VertexBuffer};
	QOpenGLBuffer m_normalbuffer{QOpenGLBuffer::VertexBuffer};
	QOpenGLBuffer m_colorbuffer{QOpenGLBuffer::VertexBuffer};

	// Shader program
	QOpenGLShaderProgram m_program;

	// 3D scene
	float m_gap = 0.2f;
	Camera m_camera;
	Light m_light;

	// widget size
	int m_startWidth = 300;
	int m_startHeight = 300;
	int m_minWidth = 150;
	int m_minHeight = 150;

};

#endif // SHADERWIDGET_H
