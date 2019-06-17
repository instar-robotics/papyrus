#include "shaderbarchart.h"

ShaderBarChart::ShaderBarChart(int xSize, int ySize):ShaderMatrix(xSize, ySize)
{
}

ShaderBarChart::~ShaderBarChart()
{
}

//Initialize the shaders during initializeGL()
void ShaderBarChart::addShaders()
{
	// Init shader program
	m_program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/shader.vert");
	m_program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/shaderbarchart.frag");

	m_program.bindAttributeLocation("in_vertex", static_cast<int>(Attribute::Vertex));
	m_program.bindAttributeLocation("in_normal", static_cast<int>(Attribute::Normal));
	m_program.bindAttributeLocation("in_color", static_cast<int>(Attribute::Color));
}

//Allocate the memory used by each vectors
void ShaderBarChart::initVectors()
{
	m_vertexes.reserve(m_xSize * m_ySize * 8);
	m_indexes.reserve(m_xSize * m_ySize * 36);
	m_colors.reserve(m_xSize * m_ySize * 8);
	m_normals.reserve(m_xSize * m_ySize * 8);
}

void ShaderBarChart::fillVectors()
{
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{
			// Vertexes
			float x = calculateXcoord(i);
			float z = calculateZcoord(j);
			QVector3D vertex;

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
			vertex.setY(calculateHeight(m_matrix[i][j]));
			vertex.setZ(z+m_edgeSize);
			m_vertexes.push_back(vertex); //front left

			vertex.setX(x+m_edgeSize);
			m_vertexes.push_back(vertex); //front right

			vertex.setZ(z-m_edgeSize);
			m_vertexes.push_back(vertex); //back right

			vertex.setX(x-m_edgeSize);
			m_vertexes.push_back(vertex); //back left

			// Colors
			QColor color = calculateColor(m_matrix[i][j], 1.0);
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
		}
	}

	// Indexes
	int current;
	for (int i = 0; i < m_ySize; i++)
	{
		for (int j = 0; j < m_xSize; j++)
		{
			current = (i * m_xSize + j)*8;

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
}

