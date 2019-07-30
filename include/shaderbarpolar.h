#ifndef SHADERBARPOLAR_H
#define SHADERBARPOLAR_H

#include "shaderpolar.h"

class ShaderBarPolar : public ShaderPolar
{
public:
	ShaderBarPolar(int xSize, int ySize, int centerIndex, RotationDir dir, MatrixReadDirection matrixReadDirection);
	~ShaderBarPolar();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

protected:
	float m_edgeSize = 0.05;
};

#endif // SHADERBARPOLAR_H
