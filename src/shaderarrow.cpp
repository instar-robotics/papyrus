#include "shaderarrow.h"

ShaderArrow::ShaderArrow()
{
	initVectors();
	fillVectors();
}

ShaderArrow::~ShaderArrow()
{
}

void ShaderArrow::initVectors()
{
	//4 triangles
	m_vertexes.reserve(4);
	m_indexes.reserve(12);
	m_colors.reserve(4);
	m_normals.reserve(4);
}


void ShaderArrow::fillVectors()
{
	clearVectors();

	/* Vertexes */
	QVector3D vertex;
	vertex.setZ(0.0);
	vertex.setY(0.0);

	vertex.setX(-m_width);
	m_vertexes.push_back(vertex);
	vertex.setX(m_width);
	m_vertexes.push_back(vertex);

	vertex.setX(0.0);
	vertex.setY(m_width);
	m_vertexes.push_back(vertex);

	vertex.setY(0.0);
	vertex.setZ(-m_length);
	m_vertexes.push_back(vertex);

	/* Colors */
	m_colors.push_back(QVector3D(0.0, 0.35, 0.0));
	m_colors.push_back(QVector3D(0.0, 0.35, 0.0));
	m_colors.push_back(QVector3D(0.0, 0.35, 0.0));
	m_colors.push_back(QVector3D(0.0, 1.0, 0.0));

	/* Normals */
	/*low vertexes*/
	m_normals.push_back(QVector3D(-0.5,0.0,0.5)); //front left
	m_normals.push_back(QVector3D(0.5,0.0,0.5)); //front right
	m_normals.push_back(QVector3D(0.0,0.5,0.5)); //front top
	/*top vertex*/
	m_normals.push_back(QVector3D(0.0,0.0,-1.0)); //direction vertex

	/* Indexes */
	//Front triangle
	m_indexes.push_back(0);
	m_indexes.push_back(1);
	m_indexes.push_back(2);

	//Left triangle
	m_indexes.push_back(0);
	m_indexes.push_back(2);
	m_indexes.push_back(3);

	//Right triangle
	m_indexes.push_back(1);
	m_indexes.push_back(2);
	m_indexes.push_back(3);

	//Down triangle
	m_indexes.push_back(0);
	m_indexes.push_back(1);
	m_indexes.push_back(3);
}
