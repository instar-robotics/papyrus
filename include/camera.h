#ifndef CAMERA_H
#define CAMERA_H

#include <QPoint>
#include <QVector4D>
#include <math.h>

#include "mathtransfo.h"


/**
 * @brief The Camera class represent the parameters of the camera in the 3d OpenGL scene. It is defined by:
 * - the direction point to which one the camera is looking at
 * - 3 rotation angles around the 3 axes with the direction point as origin
 * - 3 translations for the direction point, respectively on the x,y,z axes with the 3d scene origin as origin
 * - a level of zoom, which is the distance between the camera and the direction point
 */

class Camera
{
public:
	Camera();
	void rotateView(int x, int y, int z);
	void translateView(float x, float y, float z);
	void updatePosition(); //Calculate the position of the camera depending on its rotations and its translations
	void setDistance(float distance);
	void initDistance(float distance);
	void reinitializeCamera(); //Set the starting parameters to the camera to reinitialize its position

	float m_distance = 40.0f;

	float m_xRot = 45.0f;
	float m_yRot = 0;
	float m_zRot = 0;
	float m_rotSpeed = 10;

	float m_xTran = 0;
	float m_yTran = 0;
	float m_zTran = 0;

	//Starting parameters (also used when reinitializing the camera)
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
