#include "openglmatrix.h"

OpenGLMatrix::OpenGLMatrix(int x_size, int y_size): m_x_size(x_size), m_y_size(y_size)
{
	initMatrix();
//	connect(&m_timer_rng, SIGNAL(timeout()), this, SLOT(refreshValues()));
//	m_timer_rng.start(m_refresh_time);
}

OpenGLMatrix::~OpenGLMatrix()
{
	for(int i = 0; i < m_x_size; i++)
	{
		delete [] m_matrix[i];
	}
	delete [] m_matrix;
}
float OpenGLMatrix::calculateXcoord(int i)
{
	return (float)(-m_x_size/2+i)+0.05f;
}

float OpenGLMatrix::calculateYcoord(int j)
{
	return (float)(-m_y_size/2+j)+0.05f;
}

void OpenGLMatrix::initMatrix()
{
	m_matrix = new float *[m_x_size];
	for(int i = 0; i < m_x_size; i++)
	{
		m_matrix[i] = new float[m_y_size];
	}
	for(int i = 0; i<m_x_size; i++){
		for(int j = 0; j<m_y_size; j++){
			m_matrix[i][j] = 0.0f;
		}
	}
}

void OpenGLMatrix::paint3dObjects()
{
	// Display the 3 axes
	glColor3d(1,0,0);
	glBegin(GL_LINES);
	    glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(1.0f, 0.0f, 0.0f);
	glEnd();

	glColor3d(0,0,1);
	glBegin(GL_LINES);
	    glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 1.0f, 0.0f);
	glEnd();

	glColor3d(0,1,0);
	glBegin(GL_LINES);
	    glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 1.0f);
	glEnd();

	float x_coord;
	float y_coord;
	glColor3d(1,1,1);
	for(int i = 0; i<m_x_size; i++){
		for(int j = 0; j<m_y_size; j++){
			x_coord = calculateXcoord(i);
			y_coord = calculateYcoord(j);
			QColor color = calculateColor(m_matrix[i][j], m_max_rng, m_min_rng);
			glColor3ub(color.red(),color.green(),color.blue());
			display3dBar(x_coord/m_distance, y_coord/m_distance, 0, m_matrix[i][j]);
		}
	}
}

void OpenGLMatrix::calculation()
{
}

void OpenGLMatrix::display3dBar(float x_center, float y_center, float bottom, float top)
{
	top*=m_range;
	y_center *= -1;
	glBegin(GL_QUADS);
	    //bottom
	    glVertex3f(x_center-0.05f, y_center-0.05f, bottom);
		glVertex3f(x_center-0.05f, y_center+0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center+0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center-0.05f, bottom);

		//front
		glVertex3f(x_center-0.05f, y_center-0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center-0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center-0.05f, top);
		glVertex3f(x_center-0.05f, y_center-0.05f, top);

		//left
		glVertex3f(x_center-0.05f, y_center-0.05f, bottom);
		glVertex3f(x_center-0.05f, y_center+0.05f, bottom);
		glVertex3f(x_center-0.05f, y_center+0.05f, top);
		glVertex3f(x_center-0.05f, y_center-0.05f, top);

		//behind
		glVertex3f(x_center-0.05f, y_center+0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center+0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center+0.05f, top);
		glVertex3f(x_center-0.05f, y_center+0.05f, top);

		//right
		glVertex3f(x_center+0.05f, y_center-0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center+0.05f, bottom);
		glVertex3f(x_center+0.05f, y_center+0.05f, top);
		glVertex3f(x_center+0.05f, y_center-0.05f, top);

		//top
		glVertex3f(x_center-0.05f, y_center-0.05f, top);
		glVertex3f(x_center-0.05f, y_center+0.05f, top);
		glVertex3f(x_center+0.05f, y_center+0.05f, top);
		glVertex3f(x_center+0.05f, y_center-0.05f, top);
	glEnd();
}

void OpenGLMatrix::refreshValues()
{
	for(int i = 0; i<m_x_size; i++){
		for(int j = 0; j<m_y_size; j++){

			m_matrix[i][j] = 0.0f;
			//m_matrix[i][j] = (float)((rand()%2001)-1000)/1000;
		}
	}
}

void OpenGLMatrix::updateValues(QVector<qreal> *values)
{
	if(values->size() == m_x_size * m_y_size)
	{
		for(int i = 0; i<m_x_size; i++){
			for(int j = 0; j<m_y_size; j++){
				m_matrix[i][j] = values->at(i*m_y_size+j);
			}
		}
	}
}

