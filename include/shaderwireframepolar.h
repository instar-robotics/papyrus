#ifndef SHADERWIREFRAMEPOLAR_H
#define SHADERWIREFRAMEPOLAR_H

#include "shaderpolar.h"

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
