#ifndef SHADERARROW_H
#define SHADERARROW_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"

class ShaderArrow
{

public:
	ShaderArrow();
	~ShaderArrow();

	void initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer);

	QVector<GLuint> indexes() const;

protected:
	void initVectors();
	void fillVectors();
	void clearVectors();

private:

	float m_width = 0.8;
	float m_length = 2.0;

	//vectors
	QVector<GLuint> m_indexes; //save the index of vertexes to use in the drawing order
	QVector<QVector3D> m_vertexes; //save the coordinates of all the vertexes for the render
	QVector<QVector3D> m_normals; //save the vertexes' normals for light gesture
	QVector<QVector3D> m_colors; //save the vertexes' colors for the render

};
#endif // SHADERARROW_H
