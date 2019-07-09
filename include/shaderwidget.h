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
#include "shaderscalecircular.h"
#include "shaderscalecylinder.h"
#include "shaderarrow.h"
#include "shaderadds.h"
#include "shaderscalepolar.h"

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
	int nbMeasuresXZ() const;
	int nbMeasuresY() const;
	ShaderScalePlanes *scalePlanes() const;
	int startWidth() const;
	void updateScale(float coef);

	ShaderScaleCircular *scaleCircular() const;

	ShaderScaleCylinder *scaleCylinder() const;

	bool matrixScale() const;

	bool circScale() const;

	bool polarScale() const;

signals:
	void repaint();

protected:
	//OpenGL
	void initGPUbuffers();
	void destroyGPUbuffers();
	void initShaders();
	virtual void addShaders();
	virtual void initVectors() = 0;
	virtual void fillVectors() = 0;
	void clearVectors();
	void updateScene();

	QColor calculateColor(float const& value, const float &max_value);

private:
	void paintGL() override;
	void resizeGL(int width, int height) override;
	void displayScale();

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
	int m_nbMeasuresXZ = 11;
	int m_nbMeasuresY = 5;

	// Scale circular
	ShaderScaleCircular *m_scaleCircular = nullptr;
	ShaderScaleCylinder *m_scaleCylinder = nullptr;
	ShaderArrow *m_shaderArrow = nullptr;

	// Scale polar
	ShaderScalePolar *m_scalePolar = nullptr;

	// Scales activated or not
	bool m_matrixScale = false;
	bool m_circScale = false;
	bool m_polarScale = false;

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

private:
	// FPS
	QTime m_timer;
	size_t m_frame_count{};
	size_t m_last_count{};

};

#endif // SHADERWIDGET_H
