#ifndef SHADERCONECHART_H
#define SHADERCONECHART_H

#include "shadermatrix.h"

class ShaderConeChart : public ShaderMatrix
{
public:
	ShaderConeChart(int xSize, int ySize);
	~ShaderConeChart();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

private:
	float m_edgeSize = 0.1;
};

#endif // SHADERCONECHART_H
