#include "shaderadds.h"

ShaderAdds::ShaderAdds()
{
}

ShaderAdds::~ShaderAdds()
{
}

void ShaderAdds::clearVectors()
{
	m_vertexes.clear();
	m_indexes.clear();
	m_colors.clear();
	m_normals.clear();
}


QVector<GLuint> ShaderAdds::indexes() const
{
	return m_indexes;
}

void ShaderAdds::initGPUbuffers(QOpenGLBuffer *indexbuffer, QOpenGLBuffer *vertexbuffer, QOpenGLBuffer *normalbuffer, QOpenGLBuffer *colorbuffer)
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
