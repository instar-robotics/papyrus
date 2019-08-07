#ifndef SHADERADDS_H
#define SHADERADDS_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>

/**
 * @brief The ShaderAdds class is the parent class used by every class that display objects on the 3d OpenGL scene
 * in addition to the 3d visu. Those objects are then used by ShaderWidget using the vectors and functions
 * provided by ShaderAdds.
 */

class ShaderAdds
{

public:
	ShaderAdds();
	virtual ~ShaderAdds();

	void initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer);

	QVector<GLuint> indexes() const;

protected:
	virtual void initVectors() = 0;
	virtual void fillVectors() = 0;
	void clearVectors(); //at each frame, clear the 4 data vector before loading new data

	//vectors
	QVector<GLuint> m_indexes; //save the index of vertexes to use in the drawing order
	QVector<QVector3D> m_vertexes; //save the coordinates of all the vertexes for the render
	QVector<QVector3D> m_normals; //save the vertexes' normals for light gesture
	QVector<QVector3D> m_colors; //save the vertexes' colors for the render

};
#endif // SHADERADDS_H
