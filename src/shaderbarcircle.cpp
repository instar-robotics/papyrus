#include "shaderbarcircle.h"

ShaderBarCircle::ShaderBarCircle(int size, int centerIndex, RotationDir dir)
    :ShaderCircular(size, centerIndex, dir)
{
}

ShaderBarCircle::~ShaderBarCircle()
{
}

//Allocate the memory used by each vectors
void ShaderBarCircle::initVectors()
{
	m_vertexes.reserve(m_size * 8);
	m_indexes.reserve(m_size * 36);
	m_colors.reserve(m_size * 8);
	m_normals.reserve(m_size * 8);
}

void ShaderBarCircle::fillVectors()
{
	m_edgeSize = m_radius*(PI/2)/(m_size/2);
	if(m_edgeSize > 0.5)m_edgeSize = 0.5;
	QVector3D vertex;
	float x;
	float z;
	int current; // Index of the first vertex of the current bar
	for(int i = 0; i < m_size; i++)
	{
		// Vertexes
		x = calculateXcoord(i);
		z = calculateZcoord(i);
		current = i*8; // Each bar has 8 vertexes

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

		/*top vertexes*/
		vertex.setY(calculateHeight(m_matrix[i]));
		vertex.setZ(z+m_edgeSize);
		m_vertexes.push_back(vertex); //front left

		vertex.setX(x+m_edgeSize);
		m_vertexes.push_back(vertex); //front right

		vertex.setZ(z-m_edgeSize);
		m_vertexes.push_back(vertex); //back right

		vertex.setX(x-m_edgeSize);
		m_vertexes.push_back(vertex); //back left

		// Colors
		QColor color = calculateColor(m_matrix[i], 1.0);
		for(int k = 0; k < 8; k++)
		{
			QVector3D rgb(color.redF(), color.greenF(), color.blueF());
			m_colors.push_back(rgb);
		}

		// Normals

		/*low vertexes*/
		m_normals.push_back(QVector3D(-0.33,-0.33,0.33)); //front left
		m_normals.push_back(QVector3D(0.33,-0.33,0.33)); //front right
		m_normals.push_back(QVector3D(0.33,-0.33,-0.33)); //back right
		m_normals.push_back(QVector3D(-0.33,-0.33,-0.33)); //back left
		/*top vertexes*/
		m_normals.push_back(QVector3D(-0.33,0.33,0.33)); //front left
		m_normals.push_back(QVector3D(0.33,0.33,0.33)); //front right
		m_normals.push_back(QVector3D(0.33,0.33,-0.33)); //back right
		m_normals.push_back(QVector3D(-0.33,0.33,-0.33)); //back left

		// Indexes
		m_indexes.push_back(current);
		m_indexes.push_back(current + 1);
		m_indexes.push_back(current + 2);

		m_indexes.push_back(current);
		m_indexes.push_back(current + 2);
		m_indexes.push_back(current + 3);

		m_indexes.push_back(current);
		m_indexes.push_back(current + 1);
		m_indexes.push_back(current + 5);

		m_indexes.push_back(current);
		m_indexes.push_back(current + 5);
		m_indexes.push_back(current + 4);

		m_indexes.push_back(current + 1);
		m_indexes.push_back(current + 2);
		m_indexes.push_back(current + 6);

		m_indexes.push_back(current + 1);
		m_indexes.push_back(current + 6);
		m_indexes.push_back(current + 5);

		m_indexes.push_back(current + 2);
		m_indexes.push_back(current + 3);
		m_indexes.push_back(current + 7);

		m_indexes.push_back(current + 2);
		m_indexes.push_back(current + 7);
		m_indexes.push_back(current + 6);

		m_indexes.push_back(current + 3);
		m_indexes.push_back(current);
		m_indexes.push_back(current + 4);

		m_indexes.push_back(current + 3);
		m_indexes.push_back(current + 4);
		m_indexes.push_back(current + 7);

		m_indexes.push_back(current + 4);
		m_indexes.push_back(current + 5);
		m_indexes.push_back(current + 6);

		m_indexes.push_back(current + 4);
		m_indexes.push_back(current + 6);
		m_indexes.push_back(current + 7);
	}
}


