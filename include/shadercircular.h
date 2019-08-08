#ifndef SHADERCIRCULAR_H
#define SHADERCIRCULAR_H

#include "shaderwidget.h"

/**
 * @brief The ShaderCircular class display vector's data (unidimensional matrix's data) by using transposing
 * indexes as angles and using each value as the height of a vertex in the 3d scene.
 */

class ShaderCircular : public ShaderWidget
{
public:
	ShaderCircular(int size, int centerIndex, RotationDir dir, int extremum);
	~ShaderCircular();
	QVector<QVariant> getParameters() override;

protected:
	void initMatrix(); //allocate memory to m_matrix and initialize it as a zero matrix
	float calculateAngle(int i);
	float calculateXcoord(int i);
	float calculateZcoord(int i);
	float calculateHeight(float value);
	virtual void updateValues(QVector<qreal>* values) override; //get matrix's data from activityfetcher
	virtual void displayScale() override; //Add the 3d scale in the scene


protected:
	float* m_matrix;
	int m_size;
	int m_centerIndex;
	RotationDir m_dir;
	float m_radius = 8;
	float m_extremum; //Max and abs(min) angle value

	// Scale
	ShaderScaleCircular *m_scaleCircular;
	ShaderScaleCylinder *m_scaleCylinder;
	ShaderArrow *m_shaderArrow;
};

#endif // SHADERCIRCULAR_H
