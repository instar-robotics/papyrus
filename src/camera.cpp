﻿#include "camera.h"

Camera::Camera(): m_position(0.0,0.0,0.0,1.0)
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

void Camera::translateView(float x, float y, float z)
{
	m_xTran += x/100.0f;
	m_yTran += y/100.0f;
	m_zTran += z/100.0f;
}

void Camera::updatePosition()
{
	m_position.setX(sin(degToRad(m_yRot)) * m_distance + m_xTran);
	m_position.setY(sin(degToRad(m_xRot)) * m_distance + m_yTran);
	m_position.setZ(-cos(degToRad(m_yRot)) * m_distance + m_zTran);
}

void Camera::initDistance(float distance)
{
	m_distance = distance;
	m_defaultDistance = distance;
}

void Camera::reinitializeCamera()
{
	m_xTran = m_defaultXTran;
	m_yTran = m_defaultYTran;
	m_zTran = m_defaultZTran;
	m_xRot = m_defaultXRot;
	m_yRot = m_defaultYRot;
	m_zRot = m_defaultYRot;
	m_distance = m_defaultDistance;
}

void Camera::setDistance(float distance)
{
	m_distance = distance;
}
