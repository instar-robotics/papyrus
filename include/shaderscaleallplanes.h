#ifndef SHADERSCALEALLPLANES_H
#define SHADERSCALEALLPLANES_H

#include "shaderscale.h"

class ShaderScaleAllPlanes: public ShaderScale
{

public:
	ShaderScaleAllPlanes(float gap, int nbMeasuresXZ, int nbMeasuresY);
	~ShaderScaleAllPlanes();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

	int m_nbMeasuresXZ; //Number of measures on the XY and the YZ planes
	float m_gap; //Distance between 2 values of the matrix in the 3d display

};
#endif // SHADERSCALEALLPLANES_H
