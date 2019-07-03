#include "shaderscalecircular.h"

ShaderScaleCircular::ShaderScaleCircular(int radius):
    m_radius(radius),
    m_secRadius(radius/1.5),
    m_thirdRadius(radius/1.15)
{
	initVectors();
	fillVectors();
}

ShaderScaleCircular::~ShaderScaleCircular()
{
}

void ShaderScaleCircular::initVectors()
{
	//Axes X and Z = 4 vertexes
	//Secondary measures (30°) = 16 vertexes
	//Tertiary measures (15°) = 24 vertexes
	//Total = 44
	m_vertexes.reserve(44);
	m_indexes.reserve(44);
	m_colors.reserve(44);
	m_normals.reserve(44);
}

void ShaderScaleCircular::fillVectors()
{
	clearVectors();

	/* Vertexes */
	QVector3D vertex;
	vertex.setY(0.0);

	// X and Z axes
	vertex.setZ(0.0);
	vertex.setX(-m_radius);
	m_vertexes.push_back(vertex);
	vertex.setX(m_radius);
	m_vertexes.push_back(vertex);

	vertex.setX(0.0);
	vertex.setZ(-m_radius);
	m_vertexes.push_back(vertex);
	vertex.setZ(m_radius);
	m_vertexes.push_back(vertex);

	//Secondary measures
	for(int i = 1; i<12; i++)
	{
		if(i%3 != 0)//Do not rewrite on X and Z axes
		{
			vertex.setX(m_radius*cos(i*PI/6));
			vertex.setZ(m_radius*sin(i*PI/6));
			m_vertexes.push_back(vertex);

			vertex.setX(m_secRadius*cos(i*PI/6));
			vertex.setZ(m_secRadius*sin(i*PI/6));
			m_vertexes.push_back(vertex);
		}
	}

	//Tertiary measures
	for(int i = 1; i < 24; i++)
	{
		if(i%2 != 0)//Do not rewrite on already written measures
		{
			vertex.setX(m_radius*cos(i*PI/12));
			vertex.setZ(m_radius*sin(i*PI/12));
			m_vertexes.push_back(vertex);

			vertex.setX(m_thirdRadius*cos(i*PI/12));
			vertex.setZ(m_thirdRadius*sin(i*PI/12));
			m_vertexes.push_back(vertex);
		}
	}

	/* Colors and normals */
	QVector3D color(0.0, 0.0, 0.0);
	QVector3D normal(0.0, 1.0, 0.0);
	for(int i = 0; i<44; i++)
	{
		// Colors
		m_colors.push_back(color);

		// Normals
		m_normals.push_back(normal);
	}

	/* Indexes */
	for(int i = 0; i<22; i++)
	{
		m_indexes.push_back(i*2);
		m_indexes.push_back(i*2 + 1);
	}
}
