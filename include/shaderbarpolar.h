#ifndef SHADERBARPOLAR_H
#define SHADERBARPOLAR_H

#include "shaderpolar.h"

/**
 * @brief The ShaderBarPolar class follows matrix's data layout of ShaderPolar. One value of the matrix
 * correspond to 8 vertexes forming a bar in the 3d OpenGL space. All of those bars then form a bar chart.
 */

class ShaderBarPolar : public ShaderPolar
{
public:
	ShaderBarPolar(int xSize, int ySize, int centerIndex, RotationDir dir, MatrixReadDirection matrixReadDirection, int extremum);
	~ShaderBarPolar();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

protected:
	float m_edgeSize = 0.05;
};

#endif // SHADERBARPOLAR_H
