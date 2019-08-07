#ifndef SHADERPOLAR_H
#define SHADERPOLAR_H

#include "shaderwidget.h"

/**
 * @brief The ShaderMatrix class display matrix's data by transposing coordinates in the matrix to polar
 * coordinates in the 3d OpenGL scene and using each value as the height of a vertex in the 3d scene. By changing
 * parameters, the user can decide if columns are used as a radius or as an angle, and inversely for rows.
 */

class ShaderPolar : public ShaderWidget
{
public:
	ShaderPolar(int xSize,int ySize, int centerIndex, RotationDir dir, MatrixReadDirection matrixReadDirection, int extremum);
	~ShaderPolar();
	QVector<QVariant> getParameters() override;

protected:
	void initMatrix(); //allocate memory to m_matrix and initialize it as a zero matrix
	float calculateAngle(int j);
	float calculateXcoord(int i, int j);
	float calculateZcoord(int i, int j);
	float calculateHeight(float value);
	virtual void updateValues(QVector<qreal>* values); //get matrix's data from activityfetcher


protected:
	float** m_matrix;
	int m_xSize; //columns = angle (theta)
	int m_ySize; //rows = radius (rho)
	int m_centerIndex; //index of the angle pointed by the arrow when the visu is displayed
	RotationDir m_dir;
	float m_radiusGap = 0.1; //gap between 2 radius value in 3d scene
	float m_radiusMin; //value of the minimum radius on the polar visu display
	float m_radiusMax; //value of the maximum radius on the polar visu display
	MatrixReadDirection m_matrixReadDirection;
	float m_extremum; //Max and abs(min) angle value
};

#endif // SHADERPOLAR_H
