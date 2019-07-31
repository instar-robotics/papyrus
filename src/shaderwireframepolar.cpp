#include "shaderwireframepolar.h"

ShaderWireframePolar::ShaderWireframePolar(int xSize,int ySize, int centerIndex, RotationDir dir,
                                           MatrixReadDirection matrixReadDirection, int extremum):
    ShaderPolar(xSize, ySize, centerIndex, dir, matrixReadDirection, extremum)
{
	drawingType = GL_LINES;
}

ShaderWireframePolar::~ShaderWireframePolar()
{
}

//Allocate the memory used by each vectors
void ShaderWireframePolar::initVectors()
{
	m_vertexes.reserve(m_xSize * m_ySize);
	if(m_extremum == degToRad(360))
		m_indexes.reserve(m_xSize * (m_ySize-1) * 4 + m_xSize*2);
	else
		m_indexes.reserve((m_xSize-1) * (m_ySize-1) * 4 + m_xSize*2);
	m_colors.reserve(m_xSize * m_ySize);
	m_normals.reserve(m_xSize * m_ySize);
}

void ShaderWireframePolar::fillVectors()
{
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{
			// Vertexes
			QVector3D vertex;
			vertex.setX(calculateXcoord(i, j));
			vertex.setY(calculateHeight(m_matrix[i][j]));
			vertex.setZ(calculateZcoord(i, j));
			m_vertexes.push_back(vertex);

			// Colors
			QColor color = calculateColor(m_matrix[i][j], 1.0);
			QVector3D rgb(color.redF(), color.greenF(), color.blueF());
			m_colors.push_back(rgb);

			// Normals
			m_normals.push_back(QVector3D(0.0, 1.0, 0.0));
		}
	}

	// Indexes
	int current;
	for (int i = 0; i < m_ySize-1; i++)
	{
		for (int j = 0; j < m_xSize-1; j++)
		{
			current = i * m_xSize + j;

			m_indexes.push_back(current);
			m_indexes.push_back(current + 1);
			m_indexes.push_back(current);
			m_indexes.push_back(current + m_xSize);
		}
		//Link the last column with the first one if the polar view is 360 degres view
		if(m_extremum == degToRad(360))
		{
			m_indexes.push_back(i * m_xSize + m_xSize-1);
			m_indexes.push_back(i * m_xSize);
		}
		//Link the last column points together, whatever it is a 360 degrees view or not
		m_indexes.push_back(i * m_xSize + m_xSize-1);
		m_indexes.push_back((i+1) * m_xSize + m_xSize-1);

	}
	for (int j = 0; j < m_xSize-1; j++)
	{
		current =  (m_ySize-1) * m_xSize + j;
		m_indexes.push_back(current);
		m_indexes.push_back(current + 1);
	}
	//Link the last point of the last column with the last point of the first one if it is a 360 degres view
	if(m_extremum == degToRad(360))
	{
		m_indexes.push_back((m_ySize-1) * m_xSize + m_xSize-1);
		m_indexes.push_back((m_ySize-1) * m_xSize);
	}
}

