#ifndef SHADERSCALECYLINDER_H
#define SHADERSCALECYLINDER_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"
#include "shaderscale.h"

/**
 * @brief The ShaderScaleCylinder class is the scale class used by ShaderPolar to display its Y axe's scale.
 * It inherits from ShaderScale  to benefits of the Y axe's rescaling gesture.
 */

class ShaderScaleCylinder: public ShaderScale
{

public:
	ShaderScaleCylinder(float radius, float range, int nbMeasuresY);
	~ShaderScaleCylinder();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;

private:

	float m_radius;
};
#endif // SHADERSCALECYLINDER_H
