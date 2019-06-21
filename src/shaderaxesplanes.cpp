#include "shaderaxesplanes.h"

ShaderAxesPlanes::ShaderAxesPlanes(int rows, int columns, float range, float gap):
    m_rows(rows),
    m_columns(columns),
    m_range(range),
    m_gap(gap)
{
	initVectors();
	fillVectors();
}

ShaderAxesPlanes::~ShaderAxesPlanes()
{
}

void ShaderAxesPlanes::initVectors()
{
	m_vertexes.reserve(12);
	m_indexes.reserve(24);
	m_colors.reserve(12);
	m_normals.reserve(12);
}

void ShaderAxesPlanes::fillVectors()
{

	/* Vertexes */
	QVector3D vertex;

	// X and Z plane
	vertex.setY(0.0);

	vertex.setX(-m_gap*m_columns/2);
	vertex.setZ(m_gap*m_rows/2);
	m_vertexes.push_back(vertex);//front left

	vertex.setX(m_gap*m_columns/2);
	m_vertexes.push_back(vertex);//front right

	vertex.setZ(-m_gap*m_rows/2);
	m_vertexes.push_back(vertex);//back right

	vertex.setX(-m_gap*m_columns/2);
	m_vertexes.push_back(vertex);//back left

	// X and Y plane
	vertex.setZ(0.0);

	vertex.setX(-m_gap*m_columns/2);
	vertex.setY(m_range);
	m_vertexes.push_back(vertex);//up left

	vertex.setX(m_gap*m_columns/2);
	m_vertexes.push_back(vertex);//up right

	vertex.setY(-m_range);
	m_vertexes.push_back(vertex);//down right

	vertex.setX(-m_gap*m_columns/2);
	m_vertexes.push_back(vertex);//down left

	// Y and Z plane
	vertex.setX(0.0);

	vertex.setZ(m_gap*m_rows/2);
	vertex.setY(m_range);
	m_vertexes.push_back(vertex);//up front

	vertex.setY(-m_range);
	m_vertexes.push_back(vertex);//down front

	vertex.setZ(-m_gap*m_rows/2);
	m_vertexes.push_back(vertex);//down back

	vertex.setY(m_range);
	m_vertexes.push_back(vertex);//up back


	/* Colors and normals */
	QVector3D color(0.0, 0.0, 0.0);
	QVector3D normal(0.0, 1.0, 0.0);
	for(int i = 0; i<12; i++)
	{
		// Colors
		m_colors.push_back(color);

		// Normals
		m_normals.push_back(normal);
	}


	/* Indexes */
	for(int i = 0; i<3; i++)
	{
		m_indexes.push_back(i*4);
		m_indexes.push_back(i*4 + 1);
		m_indexes.push_back(i*4 + 1);
		m_indexes.push_back(i*4 + 2);
		m_indexes.push_back(i*4 + 2);
		m_indexes.push_back(i*4 + 3);
		m_indexes.push_back(i*4 + 3);
		m_indexes.push_back(i*4);
	}

}

QVector<GLuint> ShaderAxesPlanes::indexes() const
{
	return m_indexes;
}

void ShaderAxesPlanes::initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer)
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
