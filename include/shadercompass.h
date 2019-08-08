#ifndef SHADERCOMPASS_H
#define SHADERCOMPASS_H

#include "shaderwidget.h"
#include "types.h"

/**
 * @brief The ShaderCompass class is a 3d OpenGL visu that displays a 3d vector received from activity fetcher.
 * It is displayed as a 3d pyramid with triangular base centered on the 3d base's origin and oriented to the
 * direction point.
 */

class ShaderCompass : public ShaderWidget
{
public:
	ShaderCompass();
	~ShaderCompass();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
	void updateNormals(); //at each frame, calculate the normal of each vertex of the compass's arrow
	void calculateStartingPositions(); //initialize the compass's arrow with a starting position. The direction point is then positionned at the origin
	void calculateDirectionAngles(); //Calculate the angle in a polar base of the direction point defined in a cartesian base
	void rotateBasePosition(); //Rotate the triangular base depending on the position of the direction point
	virtual void updateValues(QVector<qreal>* values) override; //get matrix's data from activityfetcher
	void initNormalsMatrix(); //allocate memory to m_triangleNormals and initialize it as a zero matrix
	void calculateRescaledDirectionPoint(); //Move the direction point in the 3d scene depending on the level of zoom
	virtual void displayScale() override; //Add the 3d scale in the scene

private:
	QVector3D m_directionPoint;
	QVector3D m_rescaledDirectionPoint;
	QVector3D m_directionAngles;
	float m_directionRadius;

	//Points of the equilateral triangle base of the arrow
	QVector3D m_a;
	QVector3D m_b;
	QVector3D m_c;

	//Starting values of the a,b,c points before any rotation
	QVector3D m_aStartingCoord;
	QVector3D m_bStartingCoord;
	QVector3D m_cStartingCoord;

	QVector3D *m_triangleNormals; //normals used for light gesture
	float m_edge = 0.25; //edge length of the equilateral triangle base
	float m_triangleRadius; //radius of the circumscribed circle of the equilateral triangle base

	//Scale
	ShaderScaleAllPlanes *m_scaleAllPlanes;

};

#endif // SHADERSURFACE_H
