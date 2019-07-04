#ifndef SHADERBARCIRCLE_H
#define SHADERBARCIRCLE_H

#include "shadercircular.h"

class ShaderBarCircle : public ShaderCircular
{
public:
	ShaderBarCircle(int size, int centerIndex, RotationDir dir);
	~ShaderBarCircle();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

protected:
	float m_edgeSize = 0.1;
};

#endif // SHADERBARCIRCLE_H
