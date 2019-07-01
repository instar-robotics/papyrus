#ifndef SHADERSCALEPLANES_H
#define SHADERSCALEPLANES_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>

class ShaderScalePlanes
{

public:
	ShaderScalePlanes(int rows, int columns, float range, float gap, int nbMeasuresXZ, int nbMeasuresY);
	~ShaderScalePlanes();

	void initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer);
	void updateScale(float max);

	QVector<GLuint> indexes() const;
	int nbMeasuresXZ() const;
	int nbMeasuresY() const;
	float max() const;

	int rows() const;

	int columns() const;

protected:
	void initVectors();
	void fillVectors();
	void clearVectors();

private:

	int m_nbMeasuresXZ; //Number of measures on the XY and the YZ planes
	int m_nbMeasuresY; //Number of measures on the Y axe
	float m_measureX; //Distance between 2 measures on the XY plane
	float m_measureZ; //Distance between 2 measures on the ZY plane
	float m_measureY; //Distance between 2 measures on the Y axe
	int m_rows; //Nb of rows in the matrix
	int m_columns; //Nb of columns in the matrix
	float m_startRange;
	float m_range; //Max height (and min for the negatives) of the 3d display
	float m_gap; //Distance between 2 values of the matrix in the 3d display
	float m_max = 1.0; //Max scale measure

	//vectors
	QVector<GLuint> m_indexes; //save the index of vertexes to use in the drawing order
	QVector<QVector3D> m_vertexes; //save the coordinates of all the vertexes for the render
	QVector<QVector3D> m_normals; //save the vertexes' normals for light gesture
	QVector<QVector3D> m_colors; //save the vertexes' colors for the render

};
#endif // SHADERSCALEPLANES_H
