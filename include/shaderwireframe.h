#ifndef SHADERWIREFRAME_H
#define SHADERWIREFRAME_H

#include "shadermatrix.h"

/**
 * @brief The ShaderWireframe class follows matrix's data layout of ShaderMatrix. One value of the matrix
 * correspond to one vertex in the 3d OpenGL space. Those vertexes are then linked together, creating lines
 * that will form a wireframe all together
 */

class ShaderWireframe : public ShaderMatrix
{
public:
	ShaderWireframe(int xSize, int ySize);
	~ShaderWireframe();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

};

#endif // SHADERWIREFRAME_H
