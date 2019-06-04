#include "shadersurface.h"

ShaderSurface::ShaderSurface(int xSize, int ySize):ShaderMatrix(xSize, ySize)
{

}

void ShaderSurface::initVectors()
{
	m_vertexes.reserve(m_xSize * m_ySize);
	m_indexes.reserve((m_xSize-1) * (m_ySize-1) * 6);
	m_colors.reserve(m_xSize * m_ySize);
	m_normals.reserve(m_xSize * m_ySize);
}

void ShaderSurface::fillVectors()
{
	// Vertexes
	m_vertexes.clear();
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{
			QVector3D vertex;
			vertex.setX(calculateXcoord(i));
			vertex.setY(calculateHeight(m_matrix[i][j]));
			vertex.setZ(calculateZcoord(j));
			m_vertexes.push_back(vertex);
		}
	}

	// Indexes
	int current;
	m_indexes.clear();
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
		}
	}

	// Colors
	m_colors.clear();
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{
//			QColor color = calculateColor(m_matrix[i][j], 1.0);
//			QColor color = calculateColor(1.0, 1.0);
//			QVector3D rgb(color.red(), color.green(), color.blue());
//			m_colors.push_back(rgb);
			QVector3D color(0.5, 0.0, 0.0);
			m_colors.push_back(color);
		}
	}


	// Normals
	m_normals.clear();
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{

			/* ----- Faire le calcul de la normal du vertex avec les triangles ----- */
//			QVector3D normal(1.0, 1.0, 1.0);
//			normal.normalize();
//			m_normals.push_back(normal);
			m_normals.push_back(vertexNormal(i,j));
		}
	}
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
	{
		QVector3D north;
		north.setX(0.0);
		north.setY(m_matrix[i-1][j] - m_matrix[i][j]);
		north.setZ(-1.0);

		QVector3D west;
		west.setX(-1.0);
		west.setY(m_matrix[i][j-1] - m_matrix[i][j]);
		west.setZ(0.0);

		normals.push_back(QVector3D::normal(north, west));
	}

	// North east triangles
	if(i>0 && j<m_xSize-1)
	{
		QVector3D north;
		north.setX(0.0);
		north.setY(m_matrix[i-1][j] - m_matrix[i][j]);
		north.setZ(-1.0);

		QVector3D north_east;
		north_east.setX(1.0);
		north_east.setY(m_matrix[i-1][j+1] - m_matrix[i][j]);
		north_east.setZ(-1.0);

		QVector3D east;
		east.setX(1.0);
		east.setY(m_matrix[i][j+1] - m_matrix[i][j]);
		east.setZ(0.0);

		normals.push_back(QVector3D::normal(north, north_east));
		normals.push_back(QVector3D::normal(north_east, east));
	}

	// South east triangle
	if(i<m_ySize-1 && j<m_xSize-1)
	{
		QVector3D south;
		south.setX(0.0);
		south.setY(m_matrix[i+1][j] - m_matrix[i][j]);
		south.setZ(1.0);

		QVector3D east;
		east.setX(1.0);
		east.setY(m_matrix[i][j+1] - m_matrix[i][j]);
		east.setZ(0.0);

		normals.push_back(QVector3D::normal(south, east));
	}

	// South west triangles
	if(i<m_ySize-1 && j>0)
	{
		QVector3D south;
		south.setX(0.0);
		south.setY(m_matrix[i+1][j] - m_matrix[i][j]);
		south.setZ(1.0);

		QVector3D south_west;
		south_west.setX(-1.0);
		south_west.setY(m_matrix[i+1][j-1] - m_matrix[i][j]);
		south_west.setZ(1.0);

		QVector3D west;
		west.setX(-1.0);
		west.setY(m_matrix[i][j-1] - m_matrix[i][j]);
		west.setZ(0.0);

		normals.push_back(QVector3D::normal(south, south_west));
		normals.push_back(QVector3D::normal(south_west, west));
	}

	for(int current = 0; current < normals.size(); current ++)
	{
		xResult += normals.at(current).x();
		yResult += normals.at(current).y();
		zResult += normals.at(current).z();
	}
	result.setX(-xResult/normals.size());
	result.setY(-yResult/normals.size());
	result.setZ(-zResult/normals.size());
	result.normalize();
	return result;
}

