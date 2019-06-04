#include "camera.h"

Camera::Camera()
{

}

void Camera::rotateView(int x, int y, int z)
{
	m_xRot += x/20.0f;
	if(m_xRot > 359)
		m_xRot -= 360;
	else if(m_xRot < 0)
		m_xRot += 360;

	m_yRot += y/20.0f;
	if(m_yRot > 359)
		m_yRot -= 360;
	else if(m_yRot < 0)
		m_yRot += 360;

	m_zRot += z/20.0f;
	if(m_zRot > 359)
		m_zRot -= 360;
	else if(m_zRot < 0)
		m_zRot += 360;
}

void Camera::translateView(int x, int y, int z)
{
	m_xTran += x/100.0f;
	m_yTran += y/100.0f;
	m_zTran += z/100.0f;
}
