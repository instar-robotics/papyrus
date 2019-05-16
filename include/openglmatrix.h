#ifndef OPENGLMATRIX_H
#define OPENGLMATRIX_H

#include <openglwidget.h>
#include <ctime>

using namespace std;

class OpenGLMatrix : public OpenGLWidget
{
	Q_OBJECT
public:
	OpenGLMatrix(int x_size, int y_size);
	~OpenGLMatrix();
	void initMatrix(int x_size, int y_size);
	virtual void paint3dObjects();
	//virtual void paintHUD();
	virtual void calculation();
	void display3dBar(float x_center, float y_center, float bottom, float top);

	float calculateXcoord(int i);
	float calculateYcoord(int j);

public slots:
	void refreshValues();

private :
	float** m_matrix;
	int m_x_size;
	int m_y_size;
	QTimer m_timer_rng;
	int m_min_rng = -1;
	int m_max_rng = 1;
	int m_refresh_time = 100;
	float m_range = 2.0f;
};

#endif // OPENGLMATRIX_H
