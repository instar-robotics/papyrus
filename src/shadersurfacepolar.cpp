﻿#include "shadersurfacepolar.h"

ShaderSurfacePolar::ShaderSurfacePolar(int xSize,int ySize, int centerIndex, RotationDir dir,
                                       MatrixReadDirection matrixReadDirection, int extremum):
    ShaderPolar(xSize, ySize, centerIndex, dir, matrixReadDirection, extremum)
{
	initNormalsMatrixes();
}

ShaderSurfacePolar::~ShaderSurfacePolar()
{
	for(int i = 0; i < m_ySize-1; i++)
	{
		delete [] m_upTriangleNormals[i];
		delete [] m_downTriangleNormals[i];
	}
	delete [] m_upTriangleNormals;
	delete [] m_downTriangleNormals;
}

//Allocate the memory used by each vectors
void ShaderSurfacePolar::initVectors()
{
	m_vertexes.reserve(m_xSize * m_ySize);
	if(m_extremum == degToRad(360))
		m_indexes.reserve((m_xSize) * (m_ySize-1) * 6);
	else
		m_indexes.reserve((m_xSize-1) * (m_ySize-1) * 6);
	m_colors.reserve(m_xSize * m_ySize);
	m_normals.reserve(m_xSize * m_ySize);
}

void ShaderSurfacePolar::fillVectors()
{
	updateNormals();
	for(int i = 0; i < m_ySize; i++)
	{
		for(int j = 0; j < m_xSize; j++)
		{
			// Vertexes
			QVector3D vertex;
			vertex.setX(calculateXcoord(i, j));
			vertex.setY(calculateHeight(m_matrix[i][j]));
			vertex.setZ(calculateZcoord(i, j));
			m_vertexes.push_back(vertex);

			// Colors
			QColor color = calculateColor(m_matrix[i][j], 1.0);
			QVector3D rgb(color.redF(), color.greenF(), color.blueF());
			m_colors.push_back(rgb);

			// Normals
			m_normals.push_back(vertexNormal(i,j));
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
		}
		//Link the last column with the first one if the polar view is 360 degres view
		if(m_extremum == degToRad(360))
		{
			current = i * m_xSize + m_xSize-1;

			m_indexes.push_back(current);
			m_indexes.push_back(current + m_xSize);
			m_indexes.push_back(i * m_xSize);

			m_indexes.push_back(i * m_xSize);
			m_indexes.push_back(current + m_xSize);
			m_indexes.push_back(i * m_xSize + m_xSize);
		}
	}
}

//Calculate the normal of each vertex which is needed for light gesture
QVector3D ShaderSurfacePolar::vertexNormal(int i, int j)
{
	QVector3D result(0.0,0.0,0.0);
	float xResult = 0.0;
	float yResult = 0.0;
	float zResult = 0.0;

	if(i < 0 || i >= m_ySize || j < 0 || j >= m_xSize+1)
	{
		qWarning() << "The vertex of coordinates (" << i << "," << j << ") does not exist";
		return result;
	}
	QVector<QVector3D> normals;
	//The normal of a vertex is the average of each triangle's normal for whom the vertex is one of the apexes

	// North west triangle
	if(i>0 && j>0)
		normals.push_back(m_downTriangleNormals[i-1][j-1]);
	else if(i>0 && j==0)
		normals.push_back(m_upTriangleNormals[i-1][m_xSize-1]);

	// North east triangles
	if(i>0)
	{
		normals.push_back(m_upTriangleNormals[i-1][j]);
		normals.push_back(m_downTriangleNormals[i-1][j]);
	}
	// South east triangle
	if(i<m_ySize-1)
		normals.push_back(m_upTriangleNormals[i][j]);

	// South west triangles
	if(i<m_ySize-1 && j>0)
	{
		normals.push_back(m_upTriangleNormals[i][j-1]);
		normals.push_back(m_downTriangleNormals[i][j-1]);
	}
	else if(i<m_ySize-1 && j==0)
	{
		normals.push_back(m_upTriangleNormals[i][m_xSize-1]);
		normals.push_back(m_downTriangleNormals[i][m_xSize-1]);
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

void ShaderSurfacePolar::initNormalsMatrixes()
{
	m_upTriangleNormals = new QVector3D *[m_ySize-1];
	m_downTriangleNormals = new QVector3D *[m_ySize-1];
	for(int i = 0; i < m_ySize-1; i++)
	{
		m_upTriangleNormals[i] = new QVector3D[m_xSize];
		m_downTriangleNormals[i] = new QVector3D[m_xSize];
	}
	for(int i = 0; i<m_ySize-1; i++){
		for(int j = 0; j<m_xSize; j++){
			m_upTriangleNormals[i][j].setX(0.0);
			m_upTriangleNormals[i][j].setY(0.0);
			m_upTriangleNormals[i][j].setZ(0.0);
			m_downTriangleNormals[i][j].setX(0.0);
			m_downTriangleNormals[i][j].setY(0.0);
			m_downTriangleNormals[i][j].setZ(0.0);
		}
	}
}

//Calculate the normal of each triangle in the scene
void ShaderSurfacePolar::updateNormals()
{
	for(int i = 0; i<m_ySize-1; i++){
		for(int j = 0; j<m_xSize-1; j++){
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
		//Triangles that link the last column and the first one
		QVector3D upSouth;
		upSouth.setX(0.0);
		upSouth.setY(m_matrix[i+1][m_xSize-1] - m_matrix[i][m_xSize-1]);
		upSouth.setZ(1.0);

		QVector3D upEast;
		upEast.setX(1.0);
		upEast.setY(m_matrix[i][0] - m_matrix[i][m_xSize-1]);
		upEast.setZ(0.0);

		m_upTriangleNormals[i][m_xSize-1] = QVector3D::normal(upSouth, upEast);

		QVector3D downWest;
		downWest.setX(-1.0);
		downWest.setY(m_matrix[i+1][m_xSize-1] - m_matrix[i+1][0]);
		downWest.setZ(0.0);

		QVector3D downNorth;
		downNorth.setX(0.0);
		downNorth.setY(m_matrix[i][0] - m_matrix[i+1][0]);
		downNorth.setZ(-1.0);

		m_downTriangleNormals[i][m_xSize-1] = QVector3D::normal(downNorth, downWest);
	}
}

