#include "threadshader.h"

ThreadShader::ThreadShader(DiagramBox* box, VisuType type, QMap<QString, QVariant> parameters)
{
	m_widget = createShaderWidget(type, box->rows(), box->cols(), parameters);
	m_shaderMoveBar = new ShaderMoveBar();
	m_proxy = new ShaderProxy(m_widget, m_shaderMoveBar, box, &m_mutex);

	if(box->displayedProxy() != nullptr)
	{
		ShaderProxy *oldProxy = box->displayedProxy();
		m_proxy->positionWidget(oldProxy->scenePos().x(), oldProxy->scenePos().y());
		m_proxy->resizeWidget(oldProxy->widget()->width(), oldProxy->widget()->height());
		delete oldProxy;
	}
	else if(box->isActivityVisuEnabled())
	{
		ActivityVisualizer *oldVis = box->activityVisualizer();
		m_proxy->positionWidget(oldVis->x(), oldVis->y());
		m_proxy->resizeWidget(oldVis->width(), oldVis->height());
		delete oldVis;
		box->setIsActivityVisuEnabled(false);
	}
	else
		m_proxy->positionWidget(box->scenePos().x(), box->scenePos().y() - m_proxy->widget()->height() - 10);

	box->setDisplayedProxy(m_proxy);
	m_shaderMoveBar->setProxy(m_proxy);
	m_shaderMoveBar->setThread(this);

	start();
}

ThreadShader::~ThreadShader()
{
}

void ThreadShader::run()
{
	while(m_running)
	{
		msleep(m_delay);
		m_mutex.unlock();
	}
	quit();
}

void ThreadShader::setRunning(bool running)
{
	m_running = running;
}

ShaderMoveBar *ThreadShader::shaderMoveBar() const
{
	return m_shaderMoveBar;
}

ShaderProxy *ThreadShader::proxy() const
{
	return m_proxy;
}
