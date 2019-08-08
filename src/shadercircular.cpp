#include "shadercircular.h"

ShaderCircular::ShaderCircular(int size, int centerIndex, RotationDir dir, int extremum):
    m_size(size),
    m_centerIndex(centerIndex),
    m_dir(dir),
    m_extremum(degToRad(extremum))
{
	initMatrix();
	m_coefSize = 4.0;
	m_camera.initDistance(35.0f);
	m_scaleCircular = new ShaderScaleCircular(m_radius, m_coefSize, m_nbMeasuresY);
	m_scaleCylinder = new ShaderScaleCylinder(m_radius, m_coefSize, m_nbMeasuresY);
	m_shaderArrow = new ShaderArrow(3.0);
}

ShaderCircular::~ShaderCircular()
{
	delete [] m_matrix;
	delete m_scaleCircular;
	delete m_scaleCylinder;
	delete m_shaderArrow;
}

QVector<QVariant> ShaderCircular::getParameters()
{
	QVector<QVariant> parameters;
	parameters.push_back(QVariant(m_dir));
	parameters.push_back(QVariant(m_centerIndex));
	parameters.push_back(QVariant(m_extremum));
	return parameters;
}

float ShaderCircular::calculateAngle(int i)
{
	if(m_dir == COUNTERCLOCKWISE)
		return (i-m_centerIndex)*m_extremum/(float)m_size;
	else
		return (m_centerIndex-i)*m_extremum/(float)m_size;
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
	return value*m_range*m_coefSize;
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

void ShaderCircular::displayScale()
{
	m_scaleCircular->initGPUbuffers(&m_indexbuffer, &m_vertexbuffer, &m_normalbuffer, &m_colorbuffer);
	m_indexbuffer.bind();
	glDrawElements(GL_LINES, m_scaleCircular->indexes().size(), GL_UNSIGNED_INT, NULL);
	m_indexbuffer.release();

	m_scaleCylinder->initGPUbuffers(&m_indexbuffer, &m_vertexbuffer, &m_normalbuffer, &m_colorbuffer);
	m_indexbuffer.bind();
	glDrawElements(GL_LINES, m_scaleCylinder->indexes().size(), GL_UNSIGNED_INT, NULL);
	m_indexbuffer.release();

	m_shaderArrow->initGPUbuffers(&m_indexbuffer, &m_vertexbuffer, &m_normalbuffer, &m_colorbuffer);
	m_indexbuffer.bind();
	glDrawElements(GL_TRIANGLES, m_shaderArrow->indexes().size(), GL_UNSIGNED_INT, NULL);
	m_indexbuffer.release();
}

