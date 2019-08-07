#ifndef SHADERWIREFRAMEPOLAR_H
#define SHADERWIREFRAMEPOLAR_H

#include "shaderpolar.h"

/**
 * @brief The ShaderWireframePolar class follows matrix's data layout of ShaderPolar. One value of the matrix
 * correspond to one vertex in the 3d OpenGL space. Those vertexes are then linked together, creating lines
 * that will form a wireframe all together
 */

class ShaderWireframePolar : public ShaderPolar
{
public:
	ShaderWireframePolar(int xSize, int ySize, int centerIndex, RotationDir dir, MatrixReadDirection matrixReadDirection, int extremum);
	~ShaderWireframePolar();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

};

#endif // SHADERWIREFRAMEPOLAR_H
