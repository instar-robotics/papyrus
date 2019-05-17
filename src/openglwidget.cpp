#include "openglwidget.h"

void OpenGLWidget::initializeGL(int const& distance,
                                   int const& xRot,
                                   int const& yRot,
                                   int const& zRot,
                                   int const& xTran,
                                   int const& yTran,
                                   int const& zTran,
                                   int const& rot_speed
                                   )
{
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f); // rgba = (0,0,0,255)

	m_distance = distance;
	m_xRot = xRot;
	m_yRot = yRot;
	m_zRot = zRot;
	m_xTran = xTran;
	m_yTran = yTran;
	m_zTran = zTran;
	m_rot_speed = rot_speed;

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
	m_timer.start(20);
	m_current_frame_count = 0;
	m_last_frame_count = 0;
	m_last_time = QTime::currentTime();
}

void OpenGLWidget::paintGL() {

	if(fpsDisplayed())
		calculateFPS();
	if(mousePosDisplayed())
		displayMousePos();
	calculation();

	QTime new_time = QTime::currentTime();
	if (m_last_time.msecsTo(new_time) >= 1000) // If time is > 1 second, save current frame count and reinitialize it
	{
		m_last_frame_count = m_current_frame_count;
		m_current_frame_count = 0;
		m_last_time = QTime::currentTime();
	}
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	start3dCameraView();
	paint3dObjects();
	end3dCameraView();

	startHUDCameraView();
	paintHUD();
	endHUDCameraView();
	emit repaint();
}

void OpenGLWidget::resizeGL(int width, int height)
{
	glViewport(0,0, width, height); // Adapt view's size to window's size
}

void OpenGLWidget::wheelEvent(QWheelEvent *event)
{
	m_distance *= 1.0 + (1.0 * (-event->delta()) / 1200.0);
}

void OpenGLWidget::mousePressEvent(QMouseEvent *event)
{
	m_last_pos = event->pos();
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_last_pos.x();
	int dy = event->y() - m_last_pos.y();

	if (event->buttons() & Qt::LeftButton)
	{
		rotateView(dy*m_rot_speed, 0, 0);
		rotateView(0, dx*m_rot_speed, 0);
	}
	if (event->buttons() & Qt::RightButton)
	{
		translateView(dx*cos(MathTransfo::degToRad(m_yRot/20)), 0, dx*sin(MathTransfo::degToRad(m_yRot/20)));
		translateView(dy*-sin(MathTransfo::degToRad(m_yRot/20)), 0, dy*cos(MathTransfo::degToRad(m_yRot/20)));
	}
	else if (event->buttons() & Qt::MiddleButton)
		translateView(dx*cos(MathTransfo::degToRad(m_yRot/20)), -dy, dx*sin(MathTransfo::degToRad(m_yRot/20)));
	m_last_pos = event->pos();
}

void OpenGLWidget::keyPressEvent(QKeyEvent* event) {
	m_keyPressed = static_cast<QKeyEvent*>(event);
	if(m_keyPressed->key()==Qt::Key_Space)
		reinitializeView();
}


void OpenGLWidget::keyReleaseEvent(QKeyEvent* event)
{
	m_keyPressed = nullptr;
}

void OpenGLWidget::rotateView(int x, int y, int z)
{
	m_xRot += x;
	m_yRot += y;
	m_zRot += z;
}

void OpenGLWidget::translateView(int x, int y, int z)
{
	m_xTran += x;
	m_yTran += y;
	m_zTran += z;
}

void OpenGLWidget::reinitializeView(){
	m_xRot = 450;
	m_yRot = 0;
	m_zRot = 0;
	m_xTran = 0.0f;
	m_yTran = 0.0f;
	m_zTran = 0.0f;
}

bool OpenGLWidget::mousePosDisplayed() const
{
	return m_mousePosDisplayed;
}

void OpenGLWidget::setMousePosDisplayed(bool mousePosDisplayed)
{
	m_mousePosDisplayed = mousePosDisplayed;
}

bool OpenGLWidget::fpsDisplayed() const
{
	return m_fpsDisplayed;
}

void OpenGLWidget::setFpsDisplayed(bool fpsDisplayed)
{
	m_fpsDisplayed = fpsDisplayed;
}

void OpenGLWidget::displayMousePos() const
{
	qDebug() << "x: " << cursor().pos().x() << " y: " << cursor().pos().y();
}

void OpenGLWidget::calculateFPS()
{
	m_current_frame_count++;
	QTime new_time = QTime::currentTime();
	if (m_last_time.msecsTo(new_time) >= 1000) // If time is > 1 second, save current frame count and reinitialize it
	{
		m_last_frame_count = m_current_frame_count;
		m_current_frame_count = 0;
		m_last_time = QTime::currentTime();
	}
	// Display nb of fps of the last second
	qDebug() << "Frames : " << m_last_frame_count;
}

void OpenGLWidget::start3dCameraView()
{
	glEnable(GL_DEPTH_TEST); // Display objects by testing if their is no objects between it and the camera
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluPerspective(60.0f, 1.0*width()/height(), 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

//	gluLookAt(	(float)m_xTran/100, (float)m_yTran/100, ((float)m_zTran/100)-m_distance,
//	            (float)m_xTran/100, (float)m_yTran/100, (float)m_zTran/100,
//	            0.0, 1.0, 0.0
//	);

	gluLookAt(	0.0, 0.0, -m_distance,
	            0.0, 0.0, 0.0,
	            0.0, 1.0, 0.0
	);

//	float tranDiagonal = sqrt(pow(m_xTran/100, 2)+pow(m_zTran/100,2));
//	float tranAngle = 0.0f;
//	if(tranDiagonal != 0)
//	{
//		if(abs(m_xTran/100) > tranDiagonal)
//			tranAngle = m_xTran/100/abs(m_xTran/100);
//		else
//			tranAngle = acos(m_xTran/100/tranDiagonal);
//	}
//	m_xPos = tranDiagonal*cos(tranAngle+MathTransfo::degToRad(m_yRot/20));
//	m_zPos = tranDiagonal*sin(tranAngle+MathTransfo::degToRad(m_yRot/20));
	//qDebug() << "angle y:" << m_yRot << "tranAngle:" << tranAngle << "tranDiagonal:" << tranDiagonal << "tran x:" << m_xTran/100 << "tran z:" << m_zTran/100;
	//qDebug() << "m_xPos:" << m_xPos << "tranAngle:" << tranAngle << "m_yRot" << m_yRot/20;
//	glRotatef(m_xRot/20, 1.0f, 0.0f, 0.0f);
//	glTranslatef(m_xPos, 0.0f, 0.0f);
//	glTranslatef(0.0f, 0.0f, m_zPos);
//	glRotatef(m_yRot/20, 0.0f, 1.0f, 0.0f);
//	glRotatef(m_zRot/20, 0.0f, 0.0f, 1.0f);
//	glTranslatef(0.0f, m_yTran/100, 0.0f);

	glRotatef(m_xRot/20, 1.0f, 0.0f, 0.0f);
	glRotatef(m_yRot/20, 0.0f, 1.0f, 0.0f);
	glRotatef(m_zRot/20, 0.0f, 0.0f, 1.0f);
	glTranslatef(0.0f, 0.0f, m_zTran/100);
	glTranslatef(m_xTran/100, 0.0f, 0.0f);
	glTranslatef(0.0f, m_yTran/100, 0.0f);

	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	glPushMatrix();
	//qDebug() << "angle x: " << m_xRot << "angle y: " << m_yRot << "angle z: " << m_zRot;
}

void OpenGLWidget::end3dCameraView()
{
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glDisable(GL_DEPTH_TEST);
	glPopMatrix();
}

void OpenGLWidget::startHUDCameraView()
{
	glEnable(GL_TEXTURE_2D);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, width(), 0, height(), -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
}

void OpenGLWidget::endHUDCameraView()
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void OpenGLWidget::paintHUD()
{
//	glColor3f(1.0,1.0,1.0);
//	glBegin(GL_QUADS);
//	    glVertex2f(50.0f,50.0f);
//		glVertex2f(50.0f,100.0f);
//		glVertex2f(100.0f,100.0f);
//		glVertex2f(100.0f,50.0f);
//	glEnd();
}

void OpenGLWidget::paint3dObjects()
{

}

void OpenGLWidget::calculation()
{

}
QColor OpenGLWidget::calculateColor(float const& value, float const& max_value, float const& min_value)
{
	//map the value between [-1; 1]
	float gap = (max_value+min_value)/2;
	float tmp = value-gap;
	tmp /= (max_value-gap);

	// Make sure the value is comprised between [-1; +1]
	float capped = tmp;
	capped = capped > 1.0 ? 1.0 : (capped < -1.0 ? -1.0 : capped);

	// Normalize the value between [0; 1] for multiplication
	float normalizedValue = (capped - 1.0) / (-2.0);

	// Compute the light factor, with a two-piece equation
	float light = 0.5;

	if (0 <= normalizedValue && normalizedValue <= 0.5)
		light = 1/(20 * (normalizedValue - 0.586)) + 0.58;
	else if (0.5 < normalizedValue && normalizedValue <= 1.0)
		light = -1/(20 * (normalizedValue - 0.414)) + 0.58;
	else {
		qWarning() << "Normalized value (" << normalizedValue << ") outside of range [0, 1]";
	}

	// Make sure the equations stay within range
	light = light > 1.0 ? 1.0 : (light < 0 ? 0 : light);

	// Create the color value from the HSV scale
	// The idea is to have positive values into warm colors,
	// negative values in the cold colors
	// and black for 0.
	int hue = 180;
	if (0 <= normalizedValue && normalizedValue <= 0.5)
		hue = normalizedValue * 120; // We want from 0 (red) to 60 (yellow)
	else if (0.5 < normalizedValue && normalizedValue <= 1.0)
		hue = 359 - normalizedValue * 120;
	else {
		qWarning() << "Normalized value (" << normalizedValue << ") outside of range [0, 1]";
	}
	return QColor::fromHsl(hue, 255, light * 255);
}
