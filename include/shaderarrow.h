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

/**
 * @brief The ShaderArrow class display a 3d arrow in the 3d OpenGL scene. it is used by ShaderCircular
 * and ShaderPolar to mark the 0 index set by the user among the view parameters.
 */

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
