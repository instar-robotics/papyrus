#include "shaderpolar.h"

ShaderPolar::ShaderPolar(int xSize, int ySize, int centerIndex, RotationDir dir, MatrixReadDirection matrixReadDirection, int extremum):
    m_centerIndex(centerIndex),
    m_dir(dir),
    m_matrixReadDirection(matrixReadDirection),
    m_extremum(degToRad(extremum))
{
	if(m_matrixReadDirection == LINE_PER_LINE)
	{
		m_xSize = xSize;
		m_ySize = ySize;
	}
	else
	{
		m_xSize = ySize;
		m_ySize = xSize;
	}
	m_radiusMin = m_xSize/100.0*(PI/2.0);

	initMatrix();
	m_coefSize = (m_xSize+m_ySize)/40.0;
	m_camera.initDistance((m_xSize+2*m_ySize)/(3*PI/1.75));
	m_radiusMax = m_radiusMin + m_radiusGap*m_ySize + 0.2;
	m_scalePolar = new ShaderScalePolar(m_radiusMin, m_radiusMax, m_coefSize);
	m_scaleCylinder = new ShaderScaleCylinder(m_radiusMax+0.05, m_coefSize, m_nbMeasuresY);
	m_shaderArrow = new ShaderArrow(m_radiusMin/1.5);
	m_polarScale = true;
}

ShaderPolar::~ShaderPolar()
{
	for(int i = 0; i < m_ySize; i++)
	{
		delete [] m_matrix[i];
	}
	delete [] m_matrix;
}

QVector<QVariant> ShaderPolar::getParameters()
{
	QVector<QVariant> parameters;
	parameters.push_back(QVariant(m_dir));
	parameters.push_back(QVariant(m_centerIndex));
	parameters.push_back(QVariant(m_matrixReadDirection));
	parameters.push_back(QVariant(m_extremum));
	return parameters;
}

float ShaderPolar::calculateAngle(int j)
{
	if(m_dir == COUNTERCLOCKWISE)
		return (j-m_centerIndex)*m_extremum/(float)m_xSize;
	else
		return (m_centerIndex-j)*m_extremum/(float)m_xSize;
}

float ShaderPolar::calculateXcoord(int i, int j)
{
	return (m_radiusMin + m_radiusGap*i)*cos(calculateAngle(j)+PI/2); // radius * cos(angle)
}

float ShaderPolar::calculateZcoord(int i, int j)
{
	//Z is reversed compared to the sinus, so we multiplie by -1
	return -(m_radiusMin + m_radiusGap*i)*sin(calculateAngle(j)+PI/2); // -radius * sin(angle)
}


float ShaderPolar::calculateHeight(float value)
{
	return value*m_range*m_coefSize;
}

void ShaderPolar::initMatrix()
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

void ShaderPolar::updateValues(QVector<qreal> *values)
{
	if(values->size() == m_xSize * m_ySize)
	{
		if(m_matrixReadDirection == LINE_PER_LINE)
			for(int i = 0; i<m_ySize; i++){
				for(int j = 0; j<m_xSize; j++){
					m_matrix[i][j] = values->at(i*m_xSize+j);
				}
			}
		else
			for(int i = 0; i<m_ySize; i++){
				for(int j = 0; j<m_xSize; j++){
					m_matrix[i][j] = values->at(i+m_xSize*j);
				}
			}
	}
}

