#ifndef SHADERCONECHART_H
#define SHADERCONECHART_H

#include "shadermatrix.h"

/**
 * @brief The ShaderConeChart class follows matrix's data layout of ShaderMatrix. One value of the matrix
 * correspond to 5 vertexes forming a base-4 cone in the 3d OpenGL space. All of those cones then form a
 * cone chart.
 */

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
