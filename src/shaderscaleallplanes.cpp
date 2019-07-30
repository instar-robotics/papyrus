#include "shaderscaleallplanes.h"

ShaderScaleAllPlanes::ShaderScaleAllPlanes(float gap, int nbMeasuresXZ, int nbMeasuresY):
    ShaderScale(gap*(nbMeasuresXZ)/2, nbMeasuresY)
{
	m_gap = gap;
	m_nbMeasuresXZ = nbMeasuresXZ;
	initVectors();
	fillVectors();
}

ShaderScaleAllPlanes::~ShaderScaleAllPlanes()
{
}

void ShaderScaleAllPlanes::initVectors()
{
	//plane XY = m_measures * 2 vertexes
	//plane ZY = m_measures * 2 vertexes
	//plane XZ = m_measures * 4 vertexes
	m_vertexes.reserve(m_nbMeasuresXZ*8 + m_nbMeasuresY*4);
	m_indexes.reserve(m_nbMeasuresXZ*8 + m_nbMeasuresY*4);
	m_colors.reserve(m_nbMeasuresXZ*8 + m_nbMeasuresY*4);
	m_normals.reserve(m_nbMeasuresXZ*8 + m_nbMeasuresY*4);
}

void ShaderScaleAllPlanes::fillVectors()
{
	clearVectors();

	/* Vertexes */
	QVector3D vertex;

	// X and Y plane
	vertex.setZ(0.0);
	for(int i = 0; i<m_nbMeasuresXZ; i++)
	{
		vertex.setX(i*m_gap - m_gap*m_nbMeasuresXZ/2 + m_gap/2);

		vertex.setY(m_range);
		m_vertexes.push_back(vertex);//up

		vertex.setY(-m_range);
		m_vertexes.push_back(vertex);//down
	}
	for(int i = 0; i<m_nbMeasuresY; i++)
	{
		vertex.setY(i*m_measureY - m_range);

		vertex.setX(-m_gap*m_nbMeasuresXZ/2 + m_gap/2);
		m_vertexes.push_back(vertex);//left

		vertex.setX(m_gap*m_nbMeasuresXZ/2 - m_gap/2);
		m_vertexes.push_back(vertex);//right
	}

	// Y and Z plane
	vertex.setX(0.0);
	for(int i = 0; i<m_nbMeasuresXZ; i++)
	{
		vertex.setZ(i*m_gap - m_gap*m_nbMeasuresXZ/2 + m_gap/2);

		vertex.setY(m_range);
		m_vertexes.push_back(vertex);//up

		vertex.setY(-m_range);
		m_vertexes.push_back(vertex);//down
	}
	for(int i = 0; i<m_nbMeasuresY; i++)
	{
		vertex.setY(i*m_measureY - m_range);

		vertex.setZ(m_gap*m_nbMeasuresXZ/2 - m_gap/2);
		m_vertexes.push_back(vertex);//front

		vertex.setZ(-m_gap*m_nbMeasuresXZ/2 + m_gap/2);
		m_vertexes.push_back(vertex);//back
	}

	// X and Z plane
	vertex.setY(0.0);
	for(int i = 0; i<m_nbMeasuresXZ; i++)
	{
		vertex.setZ(i*m_gap - m_gap*m_nbMeasuresXZ/2 + m_gap/2);

		vertex.setX(-m_gap*m_nbMeasuresXZ/2 + m_gap/2);
		m_vertexes.push_back(vertex);//left

		vertex.setX(m_gap*m_nbMeasuresXZ/2 - m_gap/2);
		m_vertexes.push_back(vertex);//right
	}
	for(int i = 0; i<m_nbMeasuresXZ; i++)
	{
		vertex.setX(i*m_gap - m_gap*m_nbMeasuresXZ/2 + m_gap/2);

		vertex.setZ(-m_gap*m_nbMeasuresXZ/2 + m_gap/2);
		m_vertexes.push_back(vertex);//front

		vertex.setZ(m_gap*m_nbMeasuresXZ/2 - m_gap/2);
		m_vertexes.push_back(vertex);//back
	}

	/* Colors and normals */
	QVector3D color(0.0, 0.0, 0.0);
	QVector3D normal(0.0, 1.0, 0.0);
	for(int i = 0; i< m_nbMeasuresXZ*8 + m_nbMeasuresY*4; i++)
	{
		// Colors
		m_colors.push_back(color);

		// Normals
		m_normals.push_back(normal);
	}

	/* Indexes */

	for(int i = 0; i<(m_nbMeasuresXZ*8+m_nbMeasuresY*4)/2; i++) //X axe
	{
		m_indexes.push_back(i*2);
		m_indexes.push_back(i*2 + 1);
	}
}

