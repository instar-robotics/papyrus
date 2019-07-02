#include "shadercrown.h"

ShaderCrown::ShaderCrown(int size, int centerIndex, RotationDir dir)
    :ShaderCircular(size, centerIndex, dir)
{
}

ShaderCrown::~ShaderCrown()
{
}

//Allocate the memory used by each vectors
void ShaderCrown::initVectors()
{
	m_vertexes.reserve(m_size * 2);
	m_indexes.reserve(m_size * 6);
	m_colors.reserve(m_size * 2);
	m_normals.reserve(m_size * 2);
}

void ShaderCrown::fillVectors()
{

	for(int i = 0; i < m_size; i++)
	{
		// Vertexes
		QVector3D vertex;
		vertex.setX(calculateXcoord(i));
		vertex.setY(calculateHeight(m_matrix[i]));
		vertex.setZ(calculateZcoord(i));
		m_vertexes.push_back(vertex);
		vertex.setY(0.0);
		m_vertexes.push_back(vertex);

		// Colors
		QColor color = calculateColor(m_matrix[i], 1.0);
		QVector3D rgb(color.redF(), color.greenF(), color.blueF());
		//There are two colors, each one corresponding to one of the two vertexes
		m_colors.push_back(rgb); //Color corresponding to the vertex with a height corresponding to the matrix's value
		m_colors.push_back(rgb); //Color corresponding to the vertex with a height of 0

		//Normals
		//There are two normals, like for colors
		m_normals.push_back(QVector3D(0.0, 1.0, 0.0));
		m_normals.push_back(QVector3D(0.0, 1.0, 0.0));
	}

	//Indexes
	int current;
	for(int i = 0; i<m_size-1; i++)
	{
		current = i*2;

		m_indexes.push_back(current);
		m_indexes.push_back(current + 1);
		m_indexes.push_back(current + 2);

		m_indexes.push_back(current + 1);
		m_indexes.push_back(current + 2);
		m_indexes.push_back(current + 3);
	}

	// The last indexes are linked to the first ones to complete the circle
	current = (m_size-1)*2;

	m_indexes.push_back(current);
	m_indexes.push_back(current + 1);
	m_indexes.push_back(0);

	m_indexes.push_back(current + 1);
	m_indexes.push_back(0);
	m_indexes.push_back(1);
}


