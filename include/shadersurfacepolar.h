#ifndef SHADERSURFACEPOLAR_H
#define SHADERSURFACEPOLAR_H

#include "shaderpolar.h"

class ShaderSurfacePolar : public ShaderPolar
{
public:
	ShaderSurfacePolar(int xSize,int ySize, int centerIndex, RotationDir dir);
	~ShaderSurfacePolar();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
	QVector3D vertexNormal(int i, int j);
	void initNormalsMatrixes();
	void updateNormals();

private:
	QVector3D**m_upTriangleNormals;
	QVector3D**m_downTriangleNormals;
};

#endif // SHADERSURFACE_H
