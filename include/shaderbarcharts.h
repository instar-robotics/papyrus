#ifndef SHADERBARCHARTS_H
#define SHADERBARCHARTS_H

#include "shadermatrix.h"

class ShaderBarCharts : public ShaderMatrix
{
public:
	ShaderBarCharts(int xSize, int ySize);
	~ShaderBarCharts();

protected:
	virtual void addShaders() override;
	virtual void initVectors() override;
	virtual void fillVectors() override;

private:
	float m_edgeSize = 0.1;
};

#endif // SHADERBARCHARTS_H
