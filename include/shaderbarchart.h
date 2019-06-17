#ifndef ShaderBarChart_H
#define ShaderBarChart_H

#include "shadermatrix.h"

class ShaderBarChart : public ShaderMatrix
{
public:
	ShaderBarChart(int xSize, int ySize);
	~ShaderBarChart();

protected:
	virtual void addShaders() override;
	virtual void initVectors() override;
	virtual void fillVectors() override;

private:
	float m_edgeSize = 0.1;
};

#endif // ShaderBarChart_H
