#ifndef SHADERSURFACE_H
#define SHADERSURFACE_H

#include "shadermatrix.h"

class ShaderSurface : public ShaderMatrix
{
public:
	ShaderSurface(int xSize, int ySize);

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
	QVector3D vertexNormal(int i, int j);
};

#endif // SHADERSURFACE_H
