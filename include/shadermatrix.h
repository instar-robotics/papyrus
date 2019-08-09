#ifndef SHADERMATRIX_H
#define SHADERMATRIX_H

#include "shaderwidget.h"

/**
 * @brief The ShaderMatrix class display matrix's data by transposing coordinates in the matrix to cartesian
 * coordinates in the 3d OpenGL scene and using each value as the height of a vertex in the 3d scene.
 */

class ShaderMatrix : public ShaderWidget
{
public:
	ShaderMatrix(int xSize, int ySize);
	~ShaderMatrix();

	ShaderScalePlanes *scalePlanes() const;

protected:
	void initMatrix(); //allocate memory to m_matrix and initialize it as a zero matrix
	float calculateXcoord(int i);
	float calculateZcoord(int j);
	float calculateHeight(float value);
	virtual void updateValues(QVector<qreal>* values) override; //get matrix's data from activityfetcher
	virtual void displayScale() override; //Add the 3d scale in the scene

protected:
	float** m_matrix;
	int m_xSize; //nb of columns
	int m_ySize; //nb of lines

	// Scale
	ShaderScalePlanes *m_scalePlanes;
};

#endif // SHADERMATRIX_H
