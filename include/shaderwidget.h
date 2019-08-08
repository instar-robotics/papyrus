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
#include "shaderscaleallplanes.h"

enum class Attribute { Vertex, Normal, Color };

/**
 * @brief The ShaderWidget class is the main class of 3d OpenGL visu. Every 3d visu classes inherits from it.
 * It provides every variables and functions needed to use OpenGL and the GPU. It also provides the
 * gesture of events received from ShaderProxy.
 */

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
	void mousePressed(QPoint pos, MouseControl mouseControl);
	void mouseMoved(QPoint pos, MouseControl mouseControl);

	int minWidth() const;
	int minHeight() const;
	int nbMeasuresXZ() const;
	int nbMeasuresY() const;
	int startWidth() const;
	void updateScale(float coef);

	virtual QVector<QVariant> getParameters();

	float range() const;

	float gap() const;

signals:
	void repaint();

protected:
	//OpenGL
	void initGPUbuffers(); //at each frame, define which data vector is used by which GPU buffer
	void initShaders(); //initialize every GPU inputs that aren't buffers (light,
	virtual void addShaders(); //load the used shaders and their inputs
	virtual void initVectors() = 0; //initialize the size of the 4 data vector send to the GPU
	virtual void fillVectors() = 0;	//at each frame, fill the 4 data vector with new data
	void clearVectors(); //at each frame, clear the 4 data vector before loading new data
	void updateScene(); //at each frame, calculate and send current scene data to the GPU (vertexes' normals and camera position)
	virtual void displayScale() = 0; //Add the 3d scale in the scene depending on the visu type
	QColor calculateColor(float const& value, const float &max_value); //Calculate the color of a vertex using a purple-blue-black-yellow-red scale

private:
	void paintGL() override;
	void resizeGL(int width, int height) override;

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

	// 3D scene
	float m_gap = 0.2f; //Distance between 2 values in the 3d space
	Camera m_camera;
	Light m_light;
	float m_range = 1.0f; //Coefficient used to modify height of data depending of scale modifications
	float m_coefSize = 4.0f; //Coefficient used to set height of data to a visible size without modifying scale
	float m_moveCameraCoef = 1.0;
	int m_nbMeasuresXZ = 11;
	int m_nbMeasuresY = 5;

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
