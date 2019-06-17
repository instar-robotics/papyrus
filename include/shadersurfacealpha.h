#ifndef SHADERSURFACEALPHA_H
#define SHADERSURFACEALPHA_H

#include "shadermatrix.h"

class ShaderSurfaceAlpha : public ShaderMatrix
{
public:
	ShaderSurfaceAlpha(int xSize, int ySize);

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
	QVector3D vertexNormal(int i, int j);
};

#endif // SHADERSURFACEALPHA_H
