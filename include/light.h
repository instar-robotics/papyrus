#ifndef LIGHT_H
#define LIGHT_H

#include <QVector4D>
#include <QDebug>
#include "mathtransfo.h"
#include <math.h>

/**
 * @brief The Light class represent the parameters of the light in the 3d OpenGL scene. It is defined by:
 * - an ambient light, basically a minimum of light provided to each vertex
 * - a diffuse light that will enlighten vertexes near the camera, just like the camera was a bulb
 */

class Light
{
public:
	Light();
	void positionLight(int y);

	QVector4D m_lightNormal;
	QVector4D m_ambientLight;
	QVector4D m_diffuseLight;
};

#endif // LIGHT_H
