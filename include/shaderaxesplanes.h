#ifndef SHADERAXESPLANES_H
#define SHADERAXESPLANES_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>

/**
 * @brief The ShaderAxesPlanes class display the contour of XY, YZ and XZ planes in the 3d OpenGL scene,
 * all intersecting at the 3d base's origin. It is deprecated but can be usefull for tests.
 */

class ShaderAxesPlanes
{

public:
	ShaderAxesPlanes(int rows, int columns, float range, float gap);
	~ShaderAxesPlanes();
	QVector<GLuint> indexes() const;
	void initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer);

protected:
	void initVectors();
	void fillVectors();

private:

	int m_rows; //Nb of rows in the matrix
	int m_columns; //Nb of columns in the matrix
	float m_range; //Max height (and min for the negatives) of the 3d display
	float m_gap; //Distance between 2 values of the matrix in the 3d display

	//vectors
	QVector<GLuint> m_indexes; //save the index of vertexes to use in the drawing order
	QVector<QVector3D> m_vertexes; //save the coordinates of all the vertexes for the render
	QVector<QVector3D> m_normals; //save the vertexes' normals for light gesture
	QVector<QVector3D> m_colors; //save the vertexes' colors for the render

};
#endif // SHADERAXESPLANES_H
