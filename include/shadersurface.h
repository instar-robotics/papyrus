#ifndef SHADERSURFACE_H
#define SHADERSURFACE_H

#include "shadermatrix.h"

/**
 * @brief The ShaderSurface class follows matrix's data layout of ShaderMatrix. One value of the matrix
 * correspond to one vertex in the 3d OpenGL space. Those vertexes are then linked together, creating triangles
 * that will form a surface all together
 */

class ShaderSurface : public ShaderMatrix
{
public:
	ShaderSurface(int xSize, int ySize);
	~ShaderSurface();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
	QVector3D vertexNormal(int i, int j);
	void initNormalsMatrixes(); //allocate memory to m_upTriangleNormals and m_downTriangleNormals and initialize them as zero matrixes
	void updateNormals(); //at each frame, calculate every vertexes' normal

private:
	QVector3D**m_upTriangleNormals;
	QVector3D**m_downTriangleNormals;
};

#endif // SHADERSURFACE_H
