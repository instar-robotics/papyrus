#include "shaderscalepolar.h"

ShaderScalePolar::ShaderScalePolar(float minRadius,  float maxRadius, float range):
    m_minRadius(minRadius/1.5),
    m_maxRadius(maxRadius),
    m_startRange(range)
{
	m_range = range*m_max;
	initVectors();
	fillVectors();
}

ShaderScalePolar::~ShaderScalePolar()
{
}

void ShaderScalePolar::initVectors()
{
	//Axes X and Z = 4 vertexes
	//Y Axe at X and Z extremum = 4*2 = 8 vertexes
	//Secondary measures (30°) = 16 vertexes
	//Total = 28
	m_nbVertexes = 28;

	m_vertexes.reserve(m_nbVertexes);
	m_indexes.reserve(m_nbVertexes);
	m_colors.reserve(m_nbVertexes);
	m_normals.reserve(m_nbVertexes);
}

void ShaderScalePolar::fillVectors()
{
	clearVectors();

	/* Vertexes */
	QVector3D vertex;

	//-Z
	vertex.setY(-m_range);
	vertex.setX(0.0);
	vertex.setZ(-m_maxRadius);
	m_vertexes.push_back(vertex);
	vertex.setY(m_range);
	m_vertexes.push_back(vertex);

	// Z
	vertex.setY(-m_range);
	vertex.setX(0.0);
	vertex.setZ(m_maxRadius);
	m_vertexes.push_back(vertex);
	vertex.setY(m_range);
	m_vertexes.push_back(vertex);

	//-X
	vertex.setY(-m_range);
	vertex.setX(-m_maxRadius);
	vertex.setZ(0.0);
	m_vertexes.push_back(vertex);
	vertex.setY(m_range);
	m_vertexes.push_back(vertex);

	// X
	vertex.setY(-m_range);
	vertex.setX(m_maxRadius);
	vertex.setZ(0.0);
	m_vertexes.push_back(vertex);
	vertex.setY(m_range);
	m_vertexes.push_back(vertex);

	vertex.setY(0.0);
	// X and Z axes
	vertex.setZ(0.0);
	vertex.setX(-m_maxRadius);
	m_vertexes.push_back(vertex);
	vertex.setX(m_maxRadius);
	m_vertexes.push_back(vertex);

	vertex.setX(0.0);
	vertex.setZ(-m_maxRadius);
	m_vertexes.push_back(vertex);
	vertex.setZ(m_maxRadius);
	m_vertexes.push_back(vertex);

	vertex.setY(0.0);
	//Secondary measures
	for(int i = 1; i<12; i++)
	{
		if(i%3 != 0)//Do not rewrite on X and Z axes
		{
			vertex.setX(m_maxRadius*cos(i*PI/6));
			vertex.setZ(m_maxRadius*sin(i*PI/6));
			m_vertexes.push_back(vertex);

			vertex.setX(m_minRadius*cos(i*PI/6));
			vertex.setZ(m_minRadius*sin(i*PI/6));
			m_vertexes.push_back(vertex);
		}
	}

	/* Colors and normals */
	QVector3D color(0.0, 0.0, 0.0);
	QVector3D normal(0.0, 1.0, 0.0);
	for(int i = 0; i<m_nbVertexes; i++)
	{
		// Colors
		m_colors.push_back(color);

		// Normals
		m_normals.push_back(normal);
	}

	/* Indexes */
	for(int i = 0; i<m_nbVertexes/2; i++)
	{
		m_indexes.push_back(i*2);
		m_indexes.push_back(i*2 + 1);
	}
}

float ShaderScalePolar::max() const
{
	return m_max;
}

void ShaderScalePolar::updateScale(float max)
{
	m_max = max;
	m_range = m_startRange * m_max;
	fillVectors();
}
