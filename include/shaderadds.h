#ifndef SHADERADDS_H
#define SHADERADDS_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>

class ShaderAdds
{

public:
	ShaderAdds();
	~ShaderAdds();

	void initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer);

	QVector<GLuint> indexes() const;

protected:
	virtual void initVectors() = 0;
	virtual void fillVectors() = 0;
	void clearVectors();

protected:
	//vectors
	QVector<GLuint> m_indexes; //save the index of vertexes to use in the drawing order
	QVector<QVector3D> m_vertexes; //save the coordinates of all the vertexes for the render
	QVector<QVector3D> m_normals; //save the vertexes' normals for light gesture
	QVector<QVector3D> m_colors; //save the vertexes' colors for the render

};
#endif // SHADERADDS_H
