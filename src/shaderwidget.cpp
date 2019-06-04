#include "shaderwidget.h"
#include <QMouseEvent>
#include <cmath>
#include <QMatrix4x4>
#include <QPainter>

ShaderWidget::ShaderWidget()
{
	resize(500, 500);

	// Time
	time.start();
}

ShaderWidget::~ShaderWidget()
{
	//    glDeleteTextures(m_textureid);
}

QSize ShaderWidget::minimumSizeHint() const
{
	return QSize(200,200);
}

QSize ShaderWidget::sizeHint() const
{
	return QSize(800,600);
}

void ShaderWidget::initializeGL()
{
	// init OpenGL
	initializeOpenGLFunctions();

	initVectors();
	fillVectors();
	initGPUbuffers();
	initShaders();

	// GL options
	glClearColor(0.52f, 0.52f, 0.52f, 1.0f);
	glEnable(GL_DEPTH_TEST);
}

void ShaderWidget::paintGL()
{
	emit repaint();
	fillVectors();
	initGPUbuffers();

	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	m_program.bind();

	positionScene();

	// Vertex VBO
	m_vertexbuffer.bind();
	m_program.enableAttributeArray(static_cast<int>(Attribute::Vertex));
	m_program.setAttributeBuffer(static_cast<int>(Attribute::Vertex), GL_FLOAT, 0, 3);
	m_vertexbuffer.release();

	// Normal VBO
	m_normalbuffer.bind();
	m_program.enableAttributeArray(static_cast<int>(Attribute::Normal));
	m_program.setAttributeBuffer(static_cast<int>(Attribute::Normal), GL_FLOAT, 0, 3);
	m_normalbuffer.release();

	// Color VBO
	m_colorbuffer.bind();
	m_program.enableAttributeArray(static_cast<int>(Attribute::Color));
	m_program.setAttributeBuffer(static_cast<int>(Attribute::Color), GL_FLOAT, 0, 3);
	m_colorbuffer.release();

	m_indexbuffer.bind();
	glDrawElements(GL_TRIANGLES, m_indexes.size(), GL_UNSIGNED_INT, NULL);
	m_indexbuffer.release();

	m_program.disableAttributeArray(static_cast<int>(Attribute::Vertex));
	m_program.disableAttributeArray(static_cast<int>(Attribute::Normal));
	m_program.disableAttributeArray(static_cast<int>(Attribute::Color));

	m_program.release();
	update();
}

void ShaderWidget::initGPUbuffers()
{
	// Vertex buffer init
	m_vertexbuffer.create();
	m_vertexbuffer.bind();
	m_vertexbuffer.allocate(m_vertexes.constData(), sizeof(QVector3D) * m_vertexes.size());
	m_vertexbuffer.release();

	// Normal buffer init
	m_normalbuffer.create();
	m_normalbuffer.bind();
	m_normalbuffer.allocate(m_normals.constData(), sizeof(QVector3D) * m_normals.size());
	m_normalbuffer.release();

	// Colors buffer init
	m_colorbuffer.create();
	m_colorbuffer.bind();
	m_colorbuffer.allocate(m_colors.constData(), sizeof(QVector3D) * m_colors.size());
	m_colorbuffer.release();

	// Indexes buffer init
	m_indexbuffer.create();
	m_indexbuffer.bind();
	m_indexbuffer.allocate(m_indexes.constData(), sizeof(GLuint) * m_indexes.size());
	m_indexbuffer.release();
}

void ShaderWidget::initShaders()
{
	// Init shader program
	m_program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/shader.vert");
	m_program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/shader.frag");

	m_program.bindAttributeLocation("in_vertex", static_cast<int>(Attribute::Vertex));
	m_program.bindAttributeLocation("in_normal", static_cast<int>(Attribute::Normal));
	m_program.bindAttributeLocation("in_color", static_cast<int>(Attribute::Color));

	m_program.link();

	// Light settings
	m_program.bind();
	m_light.positionLight(m_camera.m_xRot, m_camera.m_yRot);
	m_program.setUniformValue("light_normal", m_light.m_lightNormal);
	m_program.setUniformValue("ambient_light", m_light.m_ambientLight);
	m_program.setUniformValue("diffuse_light", m_light.m_diffuseLight);

	m_program.release();
}

void ShaderWidget::positionScene()
{
	// Model view proj matrices
	QMatrix4x4 projection;
	projection.perspective(30.0f, 1.0f * width() / height(), 0.1f, 100.0f);

	QMatrix4x4 direction;
	direction.lookAt(QVector3D(0.0, 0.0, -m_camera.m_distance),
	                 QVector3D(0.0, 0.0, 0.0),
	                 QVector3D(0.0, 1.0, 0.0));

	QMatrix4x4 rotation;
	rotation.rotate(m_camera.m_xRot, QVector3D(-1.0f, 0.0f, 0.0f));
	rotation.rotate(m_camera.m_yRot, QVector3D(0.0f, 1.0f, 0.0f));
	rotation.rotate(m_camera.m_zRot, QVector3D(0.0f, 0.0f, 1.0f));

	QMatrix4x4 translation;
	translation.translate(-m_camera.m_xTran, 0.0f, 0.0f);
	translation.translate(0.0f, m_camera.m_yTran, 0.0f);
	translation.translate(0.0f, 0.0f, -m_camera.m_zTran);

	// Send the matrix's position in space to the shaders
	m_program.setUniformValue("matrix_position", projection * direction * rotation * translation);

	// Send the orientation of source light to the shaders
	m_program.setUniformValue("light_normal", m_light.m_lightNormal);
}

void ShaderWidget::resizeGL(int width, int height)
{
	glViewport(0, 0, width, height);
}

void ShaderWidget::keyPressEvent(QKeyEvent *event)
{
	if (event->key() == Qt::Key_Up)
	{
	}

}

void ShaderWidget::mousePressEvent(QMouseEvent *event)
{
	m_camera.m_lastPos = event->pos();
}

void ShaderWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_camera.m_lastPos.x();
	int dy = event->y() - m_camera.m_lastPos.y();

	if (event->buttons() & Qt::LeftButton)
	{
		m_camera.rotateView(dy*m_camera.m_rotSpeed, 0, 0);
		m_camera.rotateView(0, dx*m_camera.m_rotSpeed, 0);
		m_light.positionLight(m_camera.m_xRot, m_camera.m_yRot);
	}
	if (event->buttons() & Qt::RightButton)
	{
		m_camera.translateView(dx*cos(MathTransfo::degToRad(m_camera.m_yRot)), 0, dx*sin(MathTransfo::degToRad(m_camera.m_yRot)));
		m_camera.translateView(dy*-sin(MathTransfo::degToRad(m_camera.m_yRot)), 0, dy*cos(MathTransfo::degToRad(m_camera.m_yRot)));
	}
	else if (event->buttons() & Qt::MiddleButton)
		m_camera.translateView(dx*cos(MathTransfo::degToRad(m_camera.m_yRot)), -dy, dx*sin(MathTransfo::degToRad(m_camera.m_yRot)));

	m_camera.m_lastPos = event->pos();
}

void ShaderWidget::wheelEvent(QWheelEvent *event)
{
	m_camera.m_distance *= 1.0 + (1.0 * (-event->delta()) / 1200.0);
}

QColor ShaderWidget::calculateColor(float const& value, float const& max_value)
{
	//map the value between [-1; 1]
	float capped;
	if(max_value <= 0.0)
		capped = 0.0;
	else if(abs(value) < max_value)
		capped = value/max_value;
	else if(value < 0.0)
		capped = -1.0;
	else
		capped = 1.0;

	// Make sure the value is comprised between [-1; +1]
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

void ShaderWidget::fillVectors()
{

}

void ShaderWidget::initVectors()
{

}

void ShaderWidget::updateValues(QVector<qreal>* values)
{

}

