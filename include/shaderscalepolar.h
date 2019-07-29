﻿#ifndef SHADERSCALEPOLAR_H
#define SHADERSCALEPOLAR_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"
#include "shaderscale.h"

class ShaderScalePolar: public ShaderScale
{

public:
	ShaderScalePolar(float minRadius, float maxRadius, float range);
	~ShaderScalePolar();

protected:
	void initVectors();
	void fillVectors();

private:

	float m_minRadius;
	float m_maxRadius; //Value used as the radius limit of angle measures
	int m_nbVertexes;
};
#endif // SHADERSCALEPOLAR_H
