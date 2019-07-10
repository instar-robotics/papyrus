#ifndef SHADERMATRIX_H
#define SHADERMATRIX_H

#include "shaderwidget.h"

class ShaderMatrix : public ShaderWidget
{
public:
	ShaderMatrix(int xSize, int ySize);
	~ShaderMatrix();

protected:
	void initMatrix();
	float calculateXcoord(int i);
	float calculateZcoord(int j);
	float calculateHeight(float value);
	virtual void updateValues(QVector<qreal>* values);


protected:
	float** m_matrix;
	int m_xSize;
	int m_ySize;
};

#endif // SHADERMATRIX_H