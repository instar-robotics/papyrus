#ifndef CAMERA_H
#define CAMERA_H

#include <QPoint>

class Camera
{
public:
	Camera();
	void rotateView(int x, int y, int z);
	void translateView(int x, int y, int z);

	float m_distance = 10.0f;

	float m_xRot = 45.0f;
	float m_yRot = 0;
	float m_zRot = 0;
	float m_rotSpeed = 10;

	float m_xTran = 0;
	float m_yTran = 0;
	float m_zTran = 0;

	QPoint m_lastPos;
};

#endif // CAMERA_H
