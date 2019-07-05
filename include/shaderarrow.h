#ifndef SHADERARROW_H
#define SHADERARROW_H

#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>
#include <QObject>
#include <math.h>

#include "mathtransfo.h"
#include "shaderadds.h"

class ShaderArrow: public ShaderAdds
{

public:
	ShaderArrow(float length);
	~ShaderArrow();

protected:
	void initVectors();
	void fillVectors();

protected:
	float m_width;
	float m_length;

};
#endif // SHADERARROW_H
