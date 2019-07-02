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


void ShaderArrow::clearVectors()
{
	m_vertexes.clear();
	m_indexes.clear();
	m_colors.clear();
	m_normals.clear();
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
	for(int i = 0; i<4; i++)
	{
		m_colors.push_back(QVector3D(0.3, 0.3, 0.3));
	}

	/* Normals */
	/*low vertexes*/
	m_normals.push_back(QVector3D(-0.5,0.0,0.5)); //front left
	m_normals.push_back(QVector3D(0.5,0.0,0.5)); //front right
	m_normals.push_back(QVector3D(0.0,0.0,-1.0)); //direction vertex
	/*top vertex*/
	m_normals.push_back(QVector3D(0.0,0.5,0.5));

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

QVector<GLuint> ShaderArrow::indexes() const
{
	return m_indexes;
}

void ShaderArrow::initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer)
{
	// Vertex buffer init
	vertexbuffer->create();
	vertexbuffer->bind();
	vertexbuffer->allocate(m_vertexes.constData(), sizeof(QVector3D) * m_vertexes.size());
	vertexbuffer->release();

	// Normal buffer init
	normalbuffer->create();
	normalbuffer->bind();
	normalbuffer->allocate(m_normals.constData(), sizeof(QVector3D) * m_normals.size());
	normalbuffer->release();

	// Colors buffer init
	colorbuffer->create();
	colorbuffer->bind();
	colorbuffer->allocate(m_colors.constData(), sizeof(QVector3D) * m_colors.size());
	colorbuffer->release();

	// Indexes buffer init
	indexbuffer->create();
	indexbuffer->bind();
	indexbuffer->allocate(m_indexes.constData(), sizeof(GLuint) * m_indexes.size());
	indexbuffer->release();
}
