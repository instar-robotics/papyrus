#include "shadermatrix.h"

ShaderMatrix::ShaderMatrix(int xSize, int ySize): m_xSize(xSize), m_ySize(ySize)
{
	initMatrix();
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
	return i*m_gap - (m_ySize/2*m_gap);
}

float ShaderMatrix::calculateZcoord(int j)
{
	return j*m_gap - (m_xSize/2*m_gap);
}

float ShaderMatrix::calculateHeight(float value)
{
	return value*m_range;
}

void ShaderMatrix::initMatrix()
{
	qDebug() << "1";
	m_matrix = new float *[m_ySize];
	for(int i = 0; i < m_ySize; i++)
	{
		m_matrix[i] = new float[m_xSize];
	}
	qDebug() << "2";
	for(int i = 0; i<m_ySize; i++){
		for(int j = 0; j<m_xSize; j++){
			m_matrix[i][j] = 0.0f;
		}
	}
	qDebug() << "3";
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

