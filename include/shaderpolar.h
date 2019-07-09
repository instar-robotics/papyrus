#ifndef SHADERPOLAR_H
#define SHADERPOLAR_H

#include "shaderwidget.h"

class ShaderPolar : public ShaderWidget
{
public:
	ShaderPolar(int xSize,int ySize, int centerIndex, RotationDir dir);
	~ShaderPolar();

protected:
	void initMatrix();
	float calculateAngle(int j);
	float calculateXcoord(int i, int j);
	float calculateZcoord(int i, int j);
	float calculateHeight(float value);
	virtual void updateValues(QVector<qreal>* values);


protected:
	float** m_matrix;
	int m_xSize; //columns = angle (theta)
	int m_ySize; //rows = radius (rho)
	int m_centerIndex;
	RotationDir m_dir;
	float m_radiusGap = 0.1;
	float m_radiusMin;
	float m_radiusMax;
};

#endif // SHADERPOLAR_H
