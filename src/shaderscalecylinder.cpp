#include "shaderscalecylinder.h"

ShaderScaleCylinder::ShaderScaleCylinder(float radius, float range, int nbMeasuresY):
    ShaderScale(range, nbMeasuresY),
    m_radius(radius-0.05)
{
	initVectors();
	fillVectors();
}

ShaderScaleCylinder::~ShaderScaleCylinder()
{
}

void ShaderScaleCylinder::initVectors()
{
	//Measures (15°) = 24 vertexes
	//We remove the 90°
	//Total = 20
	m_vertexes.reserve(24 * m_nbMeasuresY);
	m_indexes.reserve(24 * m_nbMeasuresY);
	m_colors.reserve(24 * m_nbMeasuresY);
	m_normals.reserve(24 * m_nbMeasuresY);
}

void ShaderScaleCylinder::fillVectors()
{
	clearVectors();

	/* Vertexes */
	QVector3D vertex;
	for(int i = 0; i <m_nbMeasuresY; i++)
	{
		vertex.setY(-m_range + i*m_measureY);

		//Secondary measures
		for(int i = 0; i<24; i++)
		{
			vertex.setX(m_radius*cos(i*PI/12));
			vertex.setZ(m_radius*sin(i*PI/12));
			m_vertexes.push_back(vertex);
		}

		/* Colors and normals */
		QVector3D color(0.0, 0.0, 0.0);
		QVector3D normal(0.0, 1.0, 0.0);
		for(int i = 0; i<24; i++)
		{
			// Colors
			m_colors.push_back(color);

			// Normals
			m_normals.push_back(normal);
		}
	}
	/* Indexes */
	for(int i = 0; i<m_nbMeasuresY; i++)
	{
		for(int j = 0; j < 23; j++)
		{
			m_indexes.push_back(i*24 + j);
			m_indexes.push_back(i*24 + j + 1);
		}
		m_indexes.push_back(i*24 + 23);
		m_indexes.push_back(i*24);
	}
}

