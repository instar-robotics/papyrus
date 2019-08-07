#ifndef SHADERSCALECIRCULAR_H
#define SHADERSCALECIRCULAR_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"
#include "shaderscale.h"

/**
 * @brief The ShaderScaleCircular class is the scale class used by ShaderCircular. It inherits from ShaderScale
 * to benefits of the Y axe's rescaling gesture.
 */

class ShaderScaleCircular: public ShaderScale
{

public:
	ShaderScaleCircular(float radius, float range, int nbMeasuresY);
	~ShaderScaleCircular();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

	float m_radius;
	float m_secRadius;
	float m_thirdRadius;

	int m_nbVertexes;
};
#endif // SHADERSCALECIRCULAR_H
