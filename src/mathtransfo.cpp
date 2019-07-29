#include "mathtransfo.h"

float radToDeg(float x)
{
	return x*180/PI;
}

float degToRad(float x)
{
	return x/180*PI;
}

float normalizeDeg(float x)
{
	return x/359;
}

QVector3D vecRotationX(QVector3D vector, float angle)
{
	float values[] = {1.0f, 0.0f, 0.0f,
	                  0.0f, cos(angle), -sin(angle),
	                  0.0f, sin(angle), cos(angle)};
	QMatrix3x3 rotMatrix(values);
	return multMatrix3x3ByVec(rotMatrix, vector);
}

QVector3D vecRotationY(QVector3D vector, float angle)
{
	float values[] = {cos(angle), 0.0f, sin(angle),
	                  0.0f, 1.0f, 0.0f,
	                  -sin(angle), 0.0f, cos(angle)};
	QMatrix3x3 rotMatrix(values);
	return multMatrix3x3ByVec(rotMatrix, vector);
}

QVector3D vecRotationZ(QVector3D vector, float angle)
{
	float values[] = {cos(angle), -sin(angle), 0.0f,
	                  sin(angle), cos(angle), 0.0f,
	                  0.0f, 0.0f, 1.0f};
	QMatrix3x3 rotMatrix(values);
	return multMatrix3x3ByVec(rotMatrix, vector);
}

QVector3D multMatrix3x3ByVec(QMatrix3x3 mat, QVector3D vec)
{
	QVector3D result;
	result.setX(mat(0,0)*vec.x() + mat(0,1)*vec.y() + mat(0,2)*vec.z());
	result.setY(mat(1,0)*vec.x() + mat(1,1)*vec.y() + mat(1,2)*vec.z());
	result.setZ(mat(2,0)*vec.x() + mat(2,1)*vec.y() + mat(2,2)*vec.z());
	return result;
}

