#ifndef CAMERA_H
#define CAMERA_H

#include <QPoint>
#include <QVector4D>
#include <math.h>

#include "mathtransfo.h"

class Camera
{
public:
	Camera();
	void rotateView(int x, int y, int z);
	void translateView(float x, float y, float z);
	void updatePosition();
	void setDistance(float distance);
	void initDistance(float distance);
	void reinitializeCamera();

	float m_distance = 40.0f;

	float m_xRot = 45.0f;
	float m_yRot = 0;
	float m_zRot = 0;
	float m_rotSpeed = 10;

	float m_xTran = 0;
	float m_yTran = 0;
	float m_zTran = 0;

	float m_defaultDistance = 40.0f;
	float m_defaultXRot = 45.0f;
	float m_defaultYRot = 0;
	float m_defaultZRot = 0;
	float m_defaultXTran = 0;
	float m_defaultYTran = 0;
	float m_defaultZTran = 0;

	QVector4D m_position;

	QPoint m_lastPos;
};

#endif // CAMERA_H
