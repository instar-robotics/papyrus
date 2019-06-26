#ifndef LIGHT_H
#define LIGHT_H

#include <QVector4D>
#include <QDebug>
#include "mathtransfo.h"
#include <math.h>

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
