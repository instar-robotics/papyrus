#include "shadermatrix.h"

ShaderMatrix::ShaderMatrix(int xSize, int ySize): m_xSize(xSize), m_ySize(ySize)
{
	initMatrix();
}

ShaderMatrix::~ShaderMatrix()
{
	for(int i = 0; i < m_xSize; i++)
	{
		delete [] m_matrix[i];
	}
	delete [] m_matrix;
}

float ShaderMatrix::calculateXcoord(int i)
{
	return i*m_gap - (m_xSize/2*m_gap);
}

float ShaderMatrix::calculateZcoord(int j)
{
	return j*m_gap - (m_ySize/2*m_gap);
}

float ShaderMatrix::calculateHeight(float value)
{
	return value*m_range;
}

void ShaderMatrix::initMatrix()
{
	m_matrix = new float *[m_xSize];
	for(int i = 0; i < m_xSize; i++)
	{
		m_matrix[i] = new float[m_ySize];
	}
	for(int i = 0; i<m_xSize; i++){
		for(int j = 0; j<m_ySize; j++){
			m_matrix[i][j] = 0.0f;
		}
	}
}

void ShaderMatrix::updateValues(QVector<qreal> *values)
{
	if(values->size() == m_xSize * m_ySize)
	{
		for(int i = 0; i<m_xSize; i++){
			for(int j = 0; j<m_ySize; j++){
				m_matrix[i][j] = values->at(i*m_ySize+j);
			}
		}
	}
}

