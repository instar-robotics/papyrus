#ifndef SHADERCOMPASS_H
#define SHADERCOMPASS_H

#include "shaderwidget.h"
#include "types.h"

class ShaderCompass : public ShaderWidget
{
public:
	ShaderCompass();
	~ShaderCompass();

protected:
	virtual void initVectors() override;
	virtual void fillVectors() override;
	void updateNormals();
	void calculateStartingPositions();
	void calculateDirectionAngles();
	void rotateBasePosition();
	virtual void updateValues(QVector<qreal>* values);
	void initNormalsMatrix();
	void calculateRescaledDirectionPoint();

private:
	QVector3D m_directionPoint;
	QVector3D m_rescaledDirectionPoint;
	QVector3D m_directionAngles;
	float m_directionRadius;

	//Points of the equilateral triangle base of the arrow
	QVector3D m_a;
	QVector3D m_b;
	QVector3D m_c;
	QVector3D m_aStartingCoord;
	QVector3D m_bStartingCoord;
	QVector3D m_cStartingCoord;
	QVector3D *m_triangleNormals; //normals used for light gesture
	float m_edge = 0.5;
	float m_triangleRadius;

};

#endif // SHADERSURFACE_H
