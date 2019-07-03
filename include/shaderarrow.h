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
	ShaderArrow();
	~ShaderArrow();

protected:
	void initVectors();
	void fillVectors();

protected:
	float m_width = 0.8;
	float m_length = 2.0;

};
#endif // SHADERARROW_H
