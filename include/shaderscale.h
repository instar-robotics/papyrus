#ifndef SHADERSCALE_H
#define SHADERSCALE_H

#include "shaderadds.h"

/**
 * @brief The ShaderScale class is the parent class used by every class that display a scale for 3d visu in
 * the 3d OpenGL scene. It inherits from ShaderAdds and add the gesture of the Y axe's scale and its rescaling.
 */

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
