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

enum class Attribute { Vertex, Normal, Color };

class ShaderWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	ShaderWidget();
	~ShaderWidget();

	QSize minimumSizeHint() const;
	QSize sizeHint() const;

	void initializeGL();
	virtual void updateValues(QVector<qreal>* values);

signals:
	void repaint();

protected:
	//OpenGL
	void paintGL() override;
	void resizeGL(int width, int height) override;

	void initGPUbuffers();
	void initShaders();
	virtual void initVectors();
	virtual void fillVectors();
	void clearVectors();
	void updateScene();
	void initGPUbuffersForWireframe();

	// Events
//    void paintEvent(QPaintEvent *event);
	void keyPressEvent(QKeyEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

	QColor calculateColor(float const& value, const float &max_value);

private:
	// FPS
	QTime m_timer;
	size_t m_frame_count{};
	size_t m_last_count{};

	Camera m_camera;
	Light m_light;

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

	float m_gap = 0.2f;

};

#endif // SHADERWIDGET_H
