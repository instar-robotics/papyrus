#ifndef SHADERSCALE_H
#define SHADERSCALE_H

#include "shaderadds.h"

class ShaderScale: public ShaderAdds
{

public:
	ShaderScale(float range, int nbMeasuresY);
	~ShaderScale();

	int nbMeasuresY() const;

protected:

protected:
	int m_nbMeasuresY; //Number of measures on the Y axe
	float m_measureY; //Distance between 2 measures on the Y axe
	float m_range; //Max height (and min for the negatives) of the 3d display
};
#endif // SHADERSCALE_H
