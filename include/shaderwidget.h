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
	void positionScene();

	// Events
//    void paintEvent(QPaintEvent *event);
	void keyPressEvent(QKeyEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;

	QColor calculateColor(float const& value, const float &max_value);

private:
	// Heightmap infos
	size_t vertices_by_x{};
	size_t vertices_by_z{};

	// FPS
	QTime time;
	size_t frame_count{};
	size_t last_count{};

	Camera m_camera;
	Light m_light;

protected:

	//Vectors
	QVector<GLuint> m_indexes;
	QVector<QVector3D> m_vertexes;
	QVector<QVector3D> m_normals;
	QVector<QVector3D> m_colors;

	// GPU Buffer
	QOpenGLBuffer m_indexbuffer{QOpenGLBuffer::IndexBuffer};
	QOpenGLBuffer m_vertexbuffer{QOpenGLBuffer::VertexBuffer};
	QOpenGLBuffer m_normalbuffer{QOpenGLBuffer::VertexBuffer};
	QOpenGLBuffer m_colorbuffer{QOpenGLBuffer::VertexBuffer};

	// Shader program
	QOpenGLShaderProgram m_program;


};

#endif // SHADERWIDGET_H
