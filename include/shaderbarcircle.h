#ifndef SHADERBARCIRCLE_H
#define SHADERBARCIRCLE_H

#include "shadercircular.h"

/**
 * @brief The ShaderBarCircle class follows matrix's data layout of ShaderCircular. One value of the matrix
 * correspond to 8 vertexes forming a bar in the 3d OpenGL space. All of those bars then form a bar chart.
 */

class ShaderBarCircle : public ShaderCircular
{
public:
	ShaderBarCircle(int size, int centerIndex, RotationDir dir, int extremum);
	~ShaderBarCircle();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

protected:
	float m_edgeSize = 0.1;
};

#endif // SHADERBARCIRCLE_H
