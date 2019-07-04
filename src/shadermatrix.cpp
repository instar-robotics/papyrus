#include "shadermatrix.h"

ShaderMatrix::ShaderMatrix(int xSize, int ySize): m_xSize(xSize), m_ySize(ySize)
{
	initMatrix();
	m_scalePlanes = new ShaderScalePlanes(m_xSize, m_ySize, m_range, m_gap, m_nbMeasuresXZ, m_nbMeasuresY);
	m_matrixScale = true;
}

ShaderMatrix::~ShaderMatrix()
{
	for(int i = 0; i < m_ySize; i++)
	{
		delete [] m_matrix[i];
	}
	delete [] m_matrix;
}

float ShaderMatrix::calculateXcoord(int i)
{
	return i*m_gap - (m_ySize*m_gap/2) + m_gap/2;
}

float ShaderMatrix::calculateZcoord(int j)
{
	return j*m_gap - (m_xSize*m_gap/2) + m_gap/2;
}

float ShaderMatrix::calculateHeight(float value)
{
	return value*m_range;
}

void ShaderMatrix::initMatrix()
{
	m_matrix = new float *[m_ySize];
	for(int i = 0; i < m_ySize; i++)
	{
		m_matrix[i] = new float[m_xSize];
	}

	for(int i = 0; i<m_ySize; i++){
		for(int j = 0; j<m_xSize; j++){
			m_matrix[i][j] = 0.0f;
		}
	}
}

void ShaderMatrix::updateValues(QVector<qreal> *values)
{
	if(values->size() == m_xSize * m_ySize)
	{
		for(int i = 0; i<m_ySize; i++){
			for(int j = 0; j<m_xSize; j++){
				m_matrix[i][j] = values->at(i*m_xSize+j);
			}
		}
	}
}

