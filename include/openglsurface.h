#ifndef OPENGLSURFACE_H
#define OPENGLSURFACE_H

#include "openglmatrix.h"


class OpenGLSurface : public OpenGLMatrix
{
	Q_OBJECT

public:
	OpenGLSurface(int x_size, int y_size);
	//~OpenGLSurface();
	virtual void paint3dObjects();
};

#endif // OPENGLSURFACE_H
