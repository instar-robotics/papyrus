#include "shadercompass.h"

ShaderCompass::ShaderCompass()
{
	calculateStartingPositions();
	initNormalsMatrix();
	calculateDirectionAngles();
	rotateBasePosition();
	m_camera.initDistance(7.0);
	m_scaleAllPlanes = new ShaderScaleAllPlanes(m_gap, m_nbMeasuresXZ, m_nbMeasuresY);
}

ShaderCompass::~ShaderCompass()
{
	delete m_triangleNormals;
	delete m_scaleAllPlanes;
}

//Allocate the memory used by each vectors
void ShaderCompass::initVectors()
{
	//4 vertexes for the arrow
	//3 vertexes for the "north" mark (-z direction)
	m_vertexes.reserve(7);
	m_indexes.reserve(15);
	m_colors.reserve(7);
	m_normals.reserve(7);
}

void ShaderCompass::fillVectors()
{
	calculateRescaledDirectionPoint();

	/* Vertexes */
	m_vertexes.push_back(m_a);
	m_vertexes.push_back(m_b);
	m_vertexes.push_back(m_c);
	m_vertexes.push_back(m_rescaledDirectionPoint);

	m_vertexes.push_back(QVector3D(-0.25, 0.0, -1.0));
	m_vertexes.push_back(QVector3D(0.25, 0.0, -1.0));
	m_vertexes.push_back(QVector3D(0.0, 0.0, -1.4));

	/* Colors */
	m_colors.push_back(QVector3D(0.35, 0.0, 0.0));
	m_colors.push_back(QVector3D(0.35, 0.0, 0.0));
	m_colors.push_back(QVector3D(0.35, 0.0, 0.0));
	m_colors.push_back(QVector3D(1.0, 0.0, 0.0));

	m_colors.push_back(QVector3D(0.0, 0.0, 0.0));
	m_colors.push_back(QVector3D(0.0, 0.0, 0.0));
	m_colors.push_back(QVector3D(0.0, 0.0, 0.0));

	/* Normals */
	for(int i = 0; i < 4; i++)
	{
		m_normals.push_back(m_triangleNormals[i]);
	}

	/* Indexes */
	//Base triangle
	m_indexes.push_back(0);
	m_indexes.push_back(1);
	m_indexes.push_back(2);

	//Front triangle
	m_indexes.push_back(0);
	m_indexes.push_back(1);
	m_indexes.push_back(3);

	//Back triangle
	m_indexes.push_back(0);
	m_indexes.push_back(2);
	m_indexes.push_back(3);

	//Down triangle
	m_indexes.push_back(1);
	m_indexes.push_back(2);
	m_indexes.push_back(3);

	//North mark
	m_indexes.push_back(4);
	m_indexes.push_back(5);
	m_indexes.push_back(6);

}


// Calculate the position of the 3 vertexes of the equilateral triangle centered on (0,0,0)
// This triangle is the base of the compass arrow and will rotate depending on the compass direction
void ShaderCompass::calculateStartingPositions()
{
	m_directionPoint = QVector3D(0.0,0.0,0.0);
	m_rescaledDirectionPoint = QVector3D(0.0,0.0,0.0);
	m_triangleRadius = m_edge/sqrt(3); //Radius of the equilateral triangle
	float h = sqrt(pow(m_triangleRadius,2) - pow(m_edge/2.0,2)); //Height of the triangle less its radius
	m_aStartingCoord = QVector3D(0.0, m_triangleRadius, 0.0); //Top vertex
	m_bStartingCoord = QVector3D(0.0, -h, m_edge/2.0); //Front bottom vertex
	m_cStartingCoord = QVector3D(0.0, -h, -m_edge/2.0);//Back bottom vertex
}

// Calculate the Z and Y rotations of the vector from (0,0,0) to the direction point
// These angles are the rotation values to get the direction point by rotating the vector (1,0,0)
void ShaderCompass::calculateDirectionAngles()
{
	m_directionRadius = sqrt(pow(m_directionPoint.x(),2) + pow(m_directionPoint.y(),2) + pow(m_directionPoint.z(),2));
	m_directionAngles.setX(0.0);
	if(m_directionRadius == 0)
	{
		m_directionAngles.setY(0.0);
		m_directionAngles.setZ(0.0);
	}
	else
	{
		if(m_directionPoint.z() <= 0.0)
		{
			m_directionAngles.setY(acos(m_directionPoint.x()/m_directionRadius));

			//Switch angle calculation for asin(z) instead of acos(x) when asin is more precise
			if(m_directionAngles.y() <= PI/4)
				m_directionAngles.setY(-asin(m_directionPoint.z()/m_directionRadius));
			else if(m_directionAngles.y() >= 3*PI/4)
				m_directionAngles.setY(PI+asin(m_directionPoint.z()/m_directionRadius));
		}
		else
		{
			m_directionAngles.setY(-acos(m_directionPoint.x()/m_directionRadius));

			//Switch angle calculation for asin(z) instead of acos(x) when asin is more precise
			if(m_directionAngles.y() >= -PI/4)
				m_directionAngles.setY(-asin(m_directionPoint.z()/m_directionRadius));
			else if(m_directionAngles.y() <= -3*PI/4)
				m_directionAngles.setY(PI+asin(m_directionPoint.z()/m_directionRadius));
		}

		if(m_directionPoint.x() >= 0.0)
			m_directionAngles.setZ(asin(m_directionPoint.y()/m_directionRadius));
		else
			m_directionAngles.setZ(asin(m_directionPoint.y()/m_directionRadius));
	}
}

// Calculate the new position of the 3 vertex of the base triangle depending on the direction point position
void ShaderCompass::rotateBasePosition()
{
	m_a = m_aStartingCoord;
	m_a = vecRotationZ(m_a, m_directionAngles.z());
	m_a = vecRotationY(m_a, m_directionAngles.y());

	m_b = m_bStartingCoord;
	m_b = vecRotationZ(m_b, m_directionAngles.z());
	m_b = vecRotationY(m_b, m_directionAngles.y());

	m_c = m_cStartingCoord;
	m_c = vecRotationZ(m_c, m_directionAngles.z());
	m_c = vecRotationY(m_c, m_directionAngles.y());
}

void ShaderCompass::initNormalsMatrix()
{
	m_triangleNormals = new QVector3D[4];
	for(int i = 0; i < 4; i++)
		m_triangleNormals[i] = QVector3D(0.0, 0.0, 0.0);
}

void ShaderCompass::calculateRescaledDirectionPoint()
{
	m_rescaledDirectionPoint.setX(m_directionPoint.x()*m_range);
	m_rescaledDirectionPoint.setY(m_directionPoint.y()*m_range);
	m_rescaledDirectionPoint.setZ(m_directionPoint.z()*m_range);
}

void ShaderCompass::displayScale()
{
	m_scaleAllPlanes->initGPUbuffers(&m_indexbuffer, &m_vertexbuffer, &m_normalbuffer, &m_colorbuffer);
	m_indexbuffer.bind();
	glDrawElements(GL_LINES, m_scaleAllPlanes->indexes().size(), GL_UNSIGNED_INT, NULL);
	m_indexbuffer.release();
}


//Calculate the normal of each triangle in the scene
void ShaderCompass::updateNormals()
{
	//d = directionPoint

	//Calculate the values of the 6 vectors composing the arrow
	//AB vector is not usefull so we don't calculate this one
	//QVector3D abVector = QVector3D(m_b.x()-m_a.x(), m_b.y()-m_a.y(), m_b.y()-m_a.y());
	QVector3D acVector = QVector3D(m_c.x()-m_a.x(), m_c.y()-m_a.y(), m_c.z()-m_a.z());
	QVector3D bcVector = QVector3D(m_c.x()-m_b.x(), m_c.y()-m_b.y(), m_c.z()-m_b.z());
	QVector3D adVector = QVector3D(m_directionPoint.x()-m_a.x(), m_directionPoint.y()-m_a.y(), m_directionPoint.z()-m_a.z());
	QVector3D bdVector = QVector3D(m_directionPoint.x()-m_b.x(), m_directionPoint.y()-m_b.y(), m_directionPoint.z()-m_b.z());
	QVector3D cdVector = QVector3D(m_directionPoint.x()-m_c.x(), m_directionPoint.y()-m_c.y(), m_directionPoint.z()-m_c.z());

	//Calculate the normals of the 4 triangles composing the arrow
	QVector3D abcNormalTriangle = QVector3D::normal(acVector, bcVector);
	QVector3D abdNormalTriangle = QVector3D::normal(adVector, bdVector);
	QVector3D acdNormalTriangle = QVector3D::normal(acVector, cdVector);
	QVector3D bcdNormalTriangle = QVector3D::normal(bcVector, cdVector);

	//Calculate the normal of the 4 vertexes composing the the arrow
	m_triangleNormals[0].setX((abcNormalTriangle.x()+abdNormalTriangle.x()+acdNormalTriangle.x())/3.0);
	m_triangleNormals[0].setY((abcNormalTriangle.y()+abdNormalTriangle.y()+acdNormalTriangle.y())/3.0);
	m_triangleNormals[0].setZ((abcNormalTriangle.z()+abdNormalTriangle.z()+acdNormalTriangle.z())/3.0);

	m_triangleNormals[1].setX((abcNormalTriangle.x()+abdNormalTriangle.x()+bcdNormalTriangle.x())/3.0);
	m_triangleNormals[1].setY((abcNormalTriangle.y()+abdNormalTriangle.y()+bcdNormalTriangle.y())/3.0);
	m_triangleNormals[1].setZ((abcNormalTriangle.z()+abdNormalTriangle.z()+bcdNormalTriangle.z())/3.0);

	m_triangleNormals[2].setX((abcNormalTriangle.x()+bcdNormalTriangle.x()+acdNormalTriangle.x())/3.0);
	m_triangleNormals[2].setY((abcNormalTriangle.y()+bcdNormalTriangle.y()+acdNormalTriangle.y())/3.0);
	m_triangleNormals[2].setZ((abcNormalTriangle.z()+bcdNormalTriangle.z()+acdNormalTriangle.z())/3.0);

	m_triangleNormals[3].setX((bcdNormalTriangle.x()+abdNormalTriangle.x()+acdNormalTriangle.x())/3.0);
	m_triangleNormals[3].setY((bcdNormalTriangle.y()+abdNormalTriangle.y()+acdNormalTriangle.y())/3.0);
	m_triangleNormals[3].setZ((bcdNormalTriangle.z()+abdNormalTriangle.z()+acdNormalTriangle.z())/3.0);
}

void ShaderCompass::updateValues(QVector<qreal> *values)
{
	if(values->size() == 3)
	{
		m_directionPoint = QVector3D(values->at(0), values->at(1), -values->at(2));
		calculateDirectionAngles();
		rotateBasePosition();
		updateNormals();
	}
}
