#ifndef SHADERCIRCULAR_H
#define SHADERCIRCULAR_H

#include "shaderwidget.h"

class ShaderCircular : public ShaderWidget
{
public:
	ShaderCircular(int size, int centerIndex, RotationDir dir);
	~ShaderCircular();

protected:
	void initMatrix();
	float calculateAngle(int i);
	float calculateXcoord(int i);
	float calculateZcoord(int i);
	float calculateHeight(float value);
	virtual void updateValues(QVector<qreal>* values);


protected:
	float* m_matrix;
	int m_size;
	int m_centerIndex;
	RotationDir m_dir;
	float m_radius = 5;
};

#endif // SHADERCIRCULAR_H
