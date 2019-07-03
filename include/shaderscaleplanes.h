#ifndef SHADERSCALEPLANES_H
#define SHADERSCALEPLANES_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>

#include "shaderadds.h"

class ShaderScalePlanes: public ShaderAdds
{

public:
	ShaderScalePlanes(int rows, int columns, float range, float gap, int nbMeasuresXZ, int nbMeasuresY);
	~ShaderScalePlanes();

	void updateScale(float max);

	int nbMeasuresXZ() const;
	int nbMeasuresY() const;
	float max() const;

	int rows() const;

	int columns() const;

protected:
	void initVectors();
	void fillVectors();

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

};
#endif // SHADERSCALEPLANES_H
