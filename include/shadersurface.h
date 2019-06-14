#ifndef SHADERSURFACE_H
#define SHADERSURFACE_H

#include "shadermatrix.h"

class ShaderSurface : public ShaderMatrix
{
public:
	ShaderSurface(int xSize, int ySize);
	~ShaderSurface();

protected:
	virtual void addShaders() override;
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
