#include "shadercircular.h"

ShaderCircular::ShaderCircular(int size, int centerIndex, RotationDir dir):
    m_size(size),
    m_centerIndex(centerIndex),
    m_dir(dir)
{
	initMatrix();
	m_scaleCircular = new ShaderScaleCircular(m_radius, m_range, m_nbMeasuresY);
	m_scaleCylinder = new ShaderScaleCylinder(m_radius, m_range, m_nbMeasuresY);
	m_shaderArrow = new ShaderArrow();
	m_circScale = true;
}

ShaderCircular::~ShaderCircular()
{
	delete [] m_matrix;
}

float ShaderCircular::calculateAngle(int i)
{
	if(m_dir == COUNTERCLOCKWISE)
	{
		if(i >= m_centerIndex)
			return 2*(i-m_centerIndex)*PI/m_size;
		else
			return 2*(m_size-(m_centerIndex-i))*PI/m_size;
	}
	else
	{
		if(i <= m_centerIndex)
			return 2*(m_centerIndex-i)*PI/m_size;
		else
			return 2*(m_size-(i-m_centerIndex))*PI/m_size;
	}
}

float ShaderCircular::calculateXcoord(int i)
{
	return m_radius*cos(calculateAngle(i)+PI/2);
}

float ShaderCircular::calculateZcoord(int i)
{
	return -m_radius*sin(calculateAngle(i)+PI/2);
}


float ShaderCircular::calculateHeight(float value)
{
	return value*m_range;
}

void ShaderCircular::initMatrix()
{
	m_matrix = new float[m_size];
	for(int i = 0; i < m_size; i++)
	{
		m_matrix[i] = 0.0f;
	}
}

void ShaderCircular::updateValues(QVector<qreal> *values)
{
	if(values->size() == m_size)
	{
		for(int i = 0; i < m_size; i++)
			m_matrix[i] = values->at(i);
	}
}

