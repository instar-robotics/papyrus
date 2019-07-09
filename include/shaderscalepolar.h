#ifndef SHADERSCALEPOLAR_H
#define SHADERSCALEPOLAR_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"
#include "shaderadds.h"

class ShaderScalePolar: public ShaderAdds
{

public:
	ShaderScalePolar(float minRadius, float maxRadius, float range);
	~ShaderScalePolar();

	void updateScale(float max);
	float max() const;

protected:
	void initVectors();
	void fillVectors();

private:

	float m_minRadius;
	float m_maxRadius; //Value used as the radius limit of angle measures

	float m_startRange;
	float m_range; //Max height (and min for the negatives) of the 3d display
	float m_max = 1.0; //Max scale measure
	int m_nbVertexes;
};
#endif // SHADERSCALEPOLAR_H
