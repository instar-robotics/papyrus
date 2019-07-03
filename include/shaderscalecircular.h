#ifndef SHADERSCALECIRCULAR_H
#define SHADERSCALECIRCULAR_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"
#include "shaderadds.h"

class ShaderScaleCircular: public ShaderAdds
{

public:
	ShaderScaleCircular(int radius);
	~ShaderScaleCircular();

protected:
	void initVectors();
	void fillVectors();

private:

	float m_radius;
	float m_secRadius;
	float m_thirdRadius;
};
#endif // SHADERSCALECIRCULAR_H
