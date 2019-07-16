#ifndef SHADERSCALEPLANES_H
#define SHADERSCALEPLANES_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>

#include "shaderscale.h"

class ShaderScalePlanes: public ShaderScale
{

public:
	ShaderScalePlanes(int rows, int columns, float range, float gap, int nbMeasuresXZ, int nbMeasuresY);
	~ShaderScalePlanes();

	int nbMeasuresXZ() const;

	int rows() const;

	int columns() const;

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

	int m_nbMeasuresXZ; //Number of measures on the XY and the YZ planes
	float m_measureX; //Distance between 2 measures on the XY plane
	float m_measureZ; //Distance between 2 measures on the ZY plane
	int m_rows; //Nb of rows in the matrix
	int m_columns; //Nb of columns in the matrix
	float m_gap; //Distance between 2 values of the matrix in the 3d display

};
#endif // SHADERSCALEPLANES_H
