#include "shaderconechart.h"

ShaderConeChart::ShaderConeChart(int xSize, int ySize):ShaderMatrix(xSize, ySize)
{
}

ShaderConeChart::~ShaderConeChart()
{
}

//Allocate the memory used by each vectors
void ShaderConeChart::initVectors()
{
	m_vertexes.reserve(m_xSize * m_ySize * 5);
	m_indexes.reserve(m_xSize * m_ySize * 12);
	m_colors.reserve(m_xSize * m_ySize * 5);
	m_normals.reserve(m_xSize * m_ySize * 5);
}

void ShaderConeChart::fillVectors()
{
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{
			// Vertexes
			float x = calculateXcoord(i);
			float z = calculateZcoord(j);
			QVector3D vertex;

			/*low vertexes*/
			vertex.setX(x-m_edgeSize);
			vertex.setY(0.0);
			vertex.setZ(z+m_edgeSize);
			m_vertexes.push_back(vertex); //front left

			vertex.setX(x+m_edgeSize);
			m_vertexes.push_back(vertex); //front right

			vertex.setZ(z-m_edgeSize);
			m_vertexes.push_back(vertex); //back right

			vertex.setX(x-m_edgeSize);
			m_vertexes.push_back(vertex); //back left

			/*top vertex*/
			vertex.setX(x);
			vertex.setY(calculateHeight(m_matrix[i][j]));
			vertex.setZ(z);
			m_vertexes.push_back(vertex);

			// Colors
			QColor color = calculateColor(m_matrix[i][j], 1.0);
			for(int k = 0; k < 4; k++)
			{
				QVector3D rgb(color.redF()/5.0, color.greenF()/5.0, color.blueF()/5.0);
				m_colors.push_back(rgb);
			}
			QVector3D rgb(color.redF(), color.greenF(), color.blueF());
			m_colors.push_back(rgb);

			// Normals

			/*low vertexes*/
			m_normals.push_back(QVector3D(-0.5,0.0,0.5)); //front left
			m_normals.push_back(QVector3D(0.5,0.0,0.5)); //front right
			m_normals.push_back(QVector3D(0.5,0.0,-0.5)); //back right
			m_normals.push_back(QVector3D(-0.5,0.0,-0.5)); //back left
			/*top vertex*/
			m_normals.push_back(QVector3D(0.0,1.0,0.0)); //front left
		}
	}

	// Indexes
	int current;
	for (int i = 0; i < m_ySize; i++)
	{
		for (int j = 0; j < m_xSize; j++)
		{
			current = (i * m_xSize + j)*5;

			m_indexes.push_back(current);
			m_indexes.push_back(current + 1);
			m_indexes.push_back(current + 4);

			m_indexes.push_back(current + 1);
			m_indexes.push_back(current + 2);
			m_indexes.push_back(current + 4);

			m_indexes.push_back(current + 2);
			m_indexes.push_back(current + 3);
			m_indexes.push_back(current + 4);

			m_indexes.push_back(current + 3);
			m_indexes.push_back(current);
			m_indexes.push_back(current + 4);
		}
	}
}

