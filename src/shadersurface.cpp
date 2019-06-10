#include "shadersurface.h"

ShaderSurface::ShaderSurface(int xSize, int ySize):ShaderMatrix(xSize, ySize)
{
	initNormalsMatrixes();
}

ShaderSurface::~ShaderSurface()
{
	for(int i = 0; i < m_xSize-1; i++)
	{
		delete [] m_upTriangleNormals[i];
		delete [] m_downTriangleNormals[i];
	}
	delete [] m_upTriangleNormals;
	delete [] m_downTriangleNormals;
}

void ShaderSurface::initVectors()
{
	m_vertexes.reserve(m_xSize * m_ySize);
	m_indexes.reserve((m_xSize-1) * (m_ySize-1) * 6);
	m_colors.reserve(m_xSize * m_ySize);
	m_normals.reserve(m_xSize * m_ySize);
//	m_wireframeVertexes.reserve(m_xSize * m_ySize);
//	m_wireframeIndexes.reserve((m_xSize-1)*(m_ySize-1)*4 + (m_xSize+m_ySize-2)*2);
//	m_wireframeColors.reserve(m_xSize * m_ySize);
}

void ShaderSurface::fillVectors()
{
	updateNormals();
//	QVector3D wireframeColor(0.0,0.0,0.0);
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
			m_normals.push_back(vertexNormal(i,j));

			/* WIREFRAME */
//			vertex.setY(vertex.y()+0.02);
//			m_wireframeVertexes.push_back(vertex);
//			m_wireframeColors.push_back(wireframeColor);
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
			m_indexes.push_back(current + m_xSize);
			m_indexes.push_back(current + 1);

			m_indexes.push_back(current + 1);
			m_indexes.push_back(current + m_xSize);
			m_indexes.push_back(current + m_xSize + 1);

			/* WIREFRAME */
//			m_wireframeIndexes.push_back(current);
//			m_wireframeIndexes.push_back(current + m_xSize);

//			m_wireframeIndexes.push_back(current);
//			m_wireframeIndexes.push_back(current + 1);
		}
	}
	/* WIREFRAME */
//	for (int i = 0; i < m_ySize-1; i++)
//	{
//		current = (i+1) * m_xSize - 1;
//		m_wireframeIndexes.push_back(current);
//		m_wireframeIndexes.push_back(current + m_xSize);
//	}
//	for (int j = 0; j < m_xSize-1; j++)
//	{
//		current = (m_ySize-1) * m_xSize + j;
//		m_wireframeIndexes.push_back(current);
//		m_wireframeIndexes.push_back(current + 1);
//	}
}

QVector3D ShaderSurface::vertexNormal(int i, int j)
{
	QVector3D result(0.0,0.0,0.0);
	float xResult = 0.0;
	float yResult = 0.0;
	float zResult = 0.0;

	if(i < 0 || i >= m_ySize || j < 0 || j >= m_xSize)
	{
		qWarning() << "The vertex of coordinates (" << i << "," << j << ") does not exist";
		return result;
	}
	QVector<QVector3D> normals;

	// North west triangle
	if(i>0 && j>0)
		normals.push_back(m_downTriangleNormals[i-1][j-1]);

	// North east triangles
	if(i>0 && j<m_xSize-1)
	{
		normals.push_back(m_upTriangleNormals[i-1][j]);
		normals.push_back(m_downTriangleNormals[i-1][j]);
	}

	// South east triangle
	if(i<m_ySize-1 && j<m_xSize-1)
		normals.push_back(m_upTriangleNormals[i][j]);

	// South west triangles
	if(i<m_ySize-1 && j>0)
	{
		normals.push_back(m_upTriangleNormals[i][j-1]);
		normals.push_back(m_downTriangleNormals[i][j-1]);
	}

	for(int current = 0; current < normals.size(); current ++)
	{
		xResult += normals.at(current).x();
		yResult += normals.at(current).y();
		zResult += normals.at(current).z();
	}
	result.setX(xResult/normals.size());
	result.setY(yResult/normals.size());
	result.setZ(-zResult/normals.size());
	result.normalize();
	return result;
}

void ShaderSurface::initNormalsMatrixes()
{
	m_upTriangleNormals = new QVector3D *[m_xSize-1];
	m_downTriangleNormals = new QVector3D *[m_xSize-1];
	for(int i = 0; i < m_xSize-1; i++)
	{
		m_upTriangleNormals[i] = new QVector3D[m_ySize-1];
		m_downTriangleNormals[i] = new QVector3D[m_ySize-1];
	}
	for(int i = 0; i<m_xSize-1; i++){
		for(int j = 0; j<m_ySize-1; j++){
			m_upTriangleNormals[i][j].setX(0.0);
			m_upTriangleNormals[i][j].setY(0.0);
			m_upTriangleNormals[i][j].setZ(0.0);
			m_downTriangleNormals[i][j].setX(0.0);
			m_downTriangleNormals[i][j].setY(0.0);
			m_downTriangleNormals[i][j].setZ(0.0);
		}
	}
}

void ShaderSurface::updateNormals()
{
	for(int i = 0; i<m_xSize-1; i++){
		for(int j = 0; j<m_ySize-1; j++){
			QVector3D upSouth;
			upSouth.setX(0.0);
			upSouth.setY(m_matrix[i+1][j] - m_matrix[i][j]);
			upSouth.setZ(1.0);

			QVector3D upEast;
			upEast.setX(1.0);
			upEast.setY(m_matrix[i][j+1] - m_matrix[i][j]);
			upEast.setZ(0.0);

			m_upTriangleNormals[i][j] = QVector3D::normal(upSouth, upEast);

			QVector3D downWest;
			downWest.setX(-1.0);
			downWest.setY(m_matrix[i+1][j] - m_matrix[i+1][j+1]);
			downWest.setZ(0.0);

			QVector3D downNorth;
			downNorth.setX(0.0);
			downNorth.setY(m_matrix[i][j+1] - m_matrix[i+1][j+1]);
			downNorth.setZ(-1.0);

			m_downTriangleNormals[i][j] = QVector3D::normal(downNorth, downWest);
		}
	}
}

