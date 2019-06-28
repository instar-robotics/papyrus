#ifndef SHADERBARCHART_H
#define SHADERBARCHART_H

#include "shadermatrix.h"

class ShaderBarChart : public ShaderMatrix
{
public:
	ShaderBarChart(int xSize, int ySize);
	~ShaderBarChart();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

private:
	float m_edgeSize = 0.1;
};

#endif // SHADERBARCHART_H
