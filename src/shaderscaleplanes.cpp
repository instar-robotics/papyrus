#include "shaderscaleplanes.h"

ShaderScalePlanes::ShaderScalePlanes(int rows, int columns, float range, float gap):
    m_rows(rows),
    m_columns(columns),
    m_range(range),
    m_gap(gap)
{

	m_measureX = columns*gap/(m_nbMeasures-1);
	m_measureZ = rows*gap/(m_nbMeasures-1);
	initVectors();
	fillVectors();
}

ShaderScalePlanes::~ShaderScalePlanes()
{
}

void ShaderScalePlanes::initVectors()
{
	//plane XY = m_measures * 2 vertexes
	//plane ZY = m_measures * 2 vertexes
	//plane XZ = 4 vertexes
	m_vertexes.reserve(4 + m_nbMeasures*4);
	m_indexes.reserve(8 + (m_nbMeasures+1)*4);
	m_colors.reserve(4 + m_nbMeasures*4);
	m_normals.reserve(4 + m_nbMeasures*4);
}

void ShaderScalePlanes::fillVectors()
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
	vertex.setZ(m_measureZ*m_nbMeasures/2 - m_measureZ/2);
	for(int i = 0; i<m_nbMeasures; i++)
	{
		vertex.setX(i*m_measureX - m_measureX*m_nbMeasures/2 + m_measureX/2);

		vertex.setY(m_range);
		m_vertexes.push_back(vertex);//up

		vertex.setY(-m_range);
		m_vertexes.push_back(vertex);//down
	}

	// Y and Z plane
	vertex.setX(m_measureX*m_nbMeasures/2 - m_measureX/2);
	for(int i = 0; i<m_nbMeasures; i++)
	{
		vertex.setZ(i*m_measureZ - m_measureZ*m_nbMeasures/2 + m_measureZ/2);

		vertex.setY(m_range);
		m_vertexes.push_back(vertex);//up

		vertex.setY(-m_range);
		m_vertexes.push_back(vertex);//down
	}

	/* Colors and normals */
	QVector3D color(0.0, 0.0, 0.0);
	QVector3D normal(0.0, 1.0, 0.0);
	for(int i = 0; i< 4+m_nbMeasures*4; i++)
	{
		// Colors
		m_colors.push_back(color);

		// Normals
		m_normals.push_back(normal);
	}

	/* Indexes */

	// X and Z plane
	m_indexes.push_back(0);
	m_indexes.push_back(1);
	m_indexes.push_back(1);
	m_indexes.push_back(2);
	m_indexes.push_back(2);
	m_indexes.push_back(3);
	m_indexes.push_back(3);
	m_indexes.push_back(0);
	for(int i = 0; i<m_nbMeasures; i++)
	{
		m_indexes.push_back(i*2 + 4);
		m_indexes.push_back(i*2 + 5);
	}
	m_indexes.push_back(4);
	m_indexes.push_back(3 + m_nbMeasures*2 -1);

	m_indexes.push_back(5);
	m_indexes.push_back(3 + m_nbMeasures*2);

	for(int i = 0; i<m_nbMeasures; i++)
	{
		m_indexes.push_back(i*2 + 4 + m_nbMeasures*2);
		m_indexes.push_back(i*2 + 5 + m_nbMeasures*2);
	}
	m_indexes.push_back(4 + m_nbMeasures*2);
	m_indexes.push_back(3 + m_nbMeasures*4 -1);

	m_indexes.push_back(5 + m_nbMeasures*2);
	m_indexes.push_back(3 + m_nbMeasures*4);
}

QVector<GLuint> ShaderScalePlanes::indexes() const
{
	return m_indexes;
}

void ShaderScalePlanes::initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer)
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
