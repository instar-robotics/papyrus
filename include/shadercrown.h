#ifndef SHADERCROWN_H
#define SHADERCROWN_H

#include "shadercircular.h"

class ShaderCrown : public ShaderCircular
{
public:
	ShaderCrown(int size, int centerIndex, RotationDir dir);
	~ShaderCrown();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
};

#endif // SHADERCROWN_H
