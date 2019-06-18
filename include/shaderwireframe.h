#ifndef SHADERWIREFRAME_H
#define SHADERWIREFRAME_H

#include "shadermatrix.h"

class ShaderWireframe : public ShaderMatrix
{
public:
	ShaderWireframe(int xSize, int ySize);
	~ShaderWireframe();

protected:
	virtual void addShaders() override;
	virtual void initVectors() override;
	virtual void fillVectors() override;

};

#endif // SHADERWIREFRAME_H
