#ifndef SHADERBARCHART_H
#define SHADERBARCHART_H

#include "shadermatrix.h"

/**
 * @brief The ShaderBarChart class follows matrix's data layout of ShaderMatrix. One value of the matrix
 * correspond to 8 vertexes forming a bar in the 3d OpenGL space. All of those bars then form a bar chart.
 */

class ShaderBarChart : public ShaderMatrix
{
public:
	ShaderBarChart(int xSize, int ySize);
	~ShaderBarChart();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

protected:
	float m_edgeSize = 0.1;
};

#endif // SHADERBARCHART_H
