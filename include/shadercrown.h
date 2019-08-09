#ifndef SHADERCROWN_H
#define SHADERCROWN_H

#include "shadercircular.h"

/**
 * @brief The ShaderCrown class follows matrix's data layout of ShaderCircular. One value of the matrix
 * correspond to a vertex in the 3d OpenGL space. Those vertexes are then linked together, creating triangles
 * that will form a surface all together. This surface is circularly disposed, forming a crown.
 */

class ShaderCrown : public ShaderCircular
{
public:
	ShaderCrown(int size, int centerIndex, RotationDir dir, int extremum);
	~ShaderCrown();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
};

#endif // SHADERCROWN_H
