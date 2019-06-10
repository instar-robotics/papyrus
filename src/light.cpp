#include "light.h"

Light::Light(): m_lightNormal(1.0, 1.0, 1.0, 1.0),
                m_ambientLight(0.5, 0.5, 0.5, 1.0),
                m_diffuseLight(1.0, 1.0, 1.0, 1.0)
{
}

void Light::positionLight(int x, int y)
{
	m_lightNormal.setX(sin(MathTransfo::degToRad(y)));
	m_lightNormal.setY(sin(MathTransfo::degToRad(x)));
	m_lightNormal.setZ(-cos(MathTransfo::degToRad(y)));
}
