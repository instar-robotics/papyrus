#ifndef SHADERSCALECYLINDER_H
#define SHADERSCALECYLINDER_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"
#include "shaderadds.h"

class ShaderScaleCylinder: public ShaderAdds
{

public:
	ShaderScaleCylinder(int radius, float range, int nbMeasuresY);
	~ShaderScaleCylinder();

	void updateScale(float max);
	float max() const;

protected:
	void initVectors();
	void fillVectors();

private:

	float m_radius;
	float m_secRadius;
	float m_thirdRadius;

	int m_nbMeasuresY; //Number of measures on the Y axe
	float m_measureY; //Distance between 2 measures on the Y axe
	float m_startRange;
	float m_range; //Max height (and min for the negatives) of the 3d display
	float m_max = 1.0; //Max scale measure
};
#endif // SHADERSCALECYLINDER_H
