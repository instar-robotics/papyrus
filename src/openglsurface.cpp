#include "openglsurface.h"

OpenGLSurface::OpenGLSurface(int x_size, int y_size):OpenGLMatrix(x_size, y_size)
{
	initMatrix();
}


void OpenGLSurface::paint3dObjects()
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

	float x_coord[4];
	float y_coord[4];
	glColor3d(1,1,1);
	for(int i = 0; i<m_x_size-1; i++){
		for(int j = 0; j<m_y_size-1; j++){
			x_coord[0] = calculateXcoord(i);
			y_coord[0] = -calculateYcoord(j);

			x_coord[1] = calculateXcoord(i);
			y_coord[1] = -calculateYcoord(j+1);

			x_coord[2] = calculateXcoord(i+1);
			y_coord[2] = -calculateYcoord(j+1);

			x_coord[3] = calculateXcoord(i+1);
			y_coord[3] = -calculateYcoord(j);
			float average = (m_matrix[i][j]+m_matrix[i][j+1]+m_matrix[i+1][j+1]+m_matrix[i+1][j])/4;
			QColor color = calculateColor(average, m_max_rng, m_min_rng);
			glColor3ub(color.red(),color.green(),color.blue());

			glBegin(GL_QUADS);
			    glVertex3f(x_coord[0], y_coord[0], m_matrix[i][j]*m_range);
				glVertex3f(x_coord[1], y_coord[1], m_matrix[i][j+1]*m_range);
				glVertex3f(x_coord[2], y_coord[2], m_matrix[i+1][j+1]*m_range);
				glVertex3f(x_coord[3], y_coord[3], m_matrix[i+1][j]*m_range);
			glEnd();
		}
	}
}
