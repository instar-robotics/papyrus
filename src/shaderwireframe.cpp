#include "shaderwireframe.h"

ShaderWireframe::ShaderWireframe(int xSize, int ySize):ShaderMatrix(xSize, ySize)
{
	drawingType = GL_LINES;
}

ShaderWireframe::~ShaderWireframe()
{
}

//Initialize the shaders during initializeGL()
void ShaderWireframe::addShaders()
{
	// Init shader program
	m_program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/shader.vert");
	m_program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/shaderbarchart.frag");

	m_program.bindAttributeLocation("in_vertex", static_cast<int>(Attribute::Vertex));
	m_program.bindAttributeLocation("in_normal", static_cast<int>(Attribute::Normal));
	m_program.bindAttributeLocation("in_color", static_cast<int>(Attribute::Color));
}

//Allocate the memory used by each vectors
void ShaderWireframe::initVectors()
{
	m_vertexes.reserve(m_xSize * m_ySize);
	m_indexes.reserve((m_xSize-1) * (m_ySize-1) * 4 + (m_xSize-1)*2 + (m_ySize-1)*2);
	m_colors.reserve(m_xSize * m_ySize);
	m_normals.reserve(m_xSize * m_ySize);
}

void ShaderWireframe::fillVectors()
{
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{
			// Vertexes
			QVector3D vertex;
			vertex.setX(calculateXcoord(i));
			vertex.setY(calculateHeight(m_matrix[i][j]));
			vertex.setZ(calculateZcoord(j));
			m_vertexes.push_back(vertex);

			// Colors
			QColor color = calculateColor(m_matrix[i][j], 1.0);
			QVector3D rgb(color.redF(), color.greenF(), color.blueF());
			m_colors.push_back(rgb);

			// Normals
			m_normals.push_back(QVector3D(0.0, 1.0, 0.0));
		}
	}

	// Indexes
	int current;
	for (int i = 0; i < m_ySize-1; i++)
	{
		for (int j = 0; j < m_xSize-1; j++)
		{
			current = i * m_xSize + j;

			m_indexes.push_back(current);
			m_indexes.push_back(current + 1);
			m_indexes.push_back(current);
			m_indexes.push_back(current + m_xSize);
		}
	}
	for (int i = 0; i < m_ySize-1; i++)
	{
		current = i*m_xSize + (m_xSize-1);
		m_indexes.push_back(current);
		m_indexes.push_back(current + m_xSize);
	}
	for (int j = 0; j < m_xSize-1; j++)
	{
		current =  (m_ySize-1) * m_xSize + j;
		m_indexes.push_back(current);
		m_indexes.push_back(current + 1);
	}
}

