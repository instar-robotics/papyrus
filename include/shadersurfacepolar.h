#ifndef SHADERSURFACEPOLAR_H
#define SHADERSURFACEPOLAR_H

#include "shaderpolar.h"

/**
 * @brief The ShaderSurfacePolar class follows matrix's data layout of ShaderPolar. One value of the matrix
 * correspond to one vertex in the 3d OpenGL space. Those vertexes are then linked together, creating triangles
 * that will form a surface all together
 */

class ShaderSurfacePolar : public ShaderPolar
{
public:
	ShaderSurfacePolar(int xSize, int ySize, int centerIndex, RotationDir dir, MatrixReadDirection matrixReadDirection, int extremum);
	~ShaderSurfacePolar();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
	QVector3D vertexNormal(int i, int j);
	void initNormalsMatrixes();//allocate memory to m_upTriangleNormals and m_downTriangleNormals and initialize them as zero matrixes
	void updateNormals(); //at each frame, calculate every vertexes' normal

private:
	QVector3D**m_upTriangleNormals;
	QVector3D**m_downTriangleNormals;
};

#endif // SHADERSURFACE_H
