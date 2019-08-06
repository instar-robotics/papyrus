#include "threadshader.h"

ThreadShader::ThreadShader(DiagramBox* box, VisuType type, QMap<QString, QVariant> parameters)
{
	m_widget = createShaderWidget(type, box->rows(), box->cols(), parameters);
	m_shaderMoveBar = new ShaderMoveBar();
	m_proxy = new ShaderProxy(m_widget, m_shaderMoveBar, box, &m_mutex);
	if(box->displayedProxy() != nullptr)
	{
		ShaderProxy *oldProxy = box->displayedProxy();
		ThreadShader *oldThread = oldProxy->moveBar()->thread();

		m_proxy->positionWidget(oldProxy->scenePos().x(), oldProxy->scenePos().y()-m_proxy->moveBarHeight());
		m_proxy->resizeWidget(oldProxy->widget()->width(), oldProxy->widget()->height());

		delete oldProxy;
		oldThread->setRunning(false);
	}
	else if(box->isActivityVisuEnabled())
	{
		ActivityVisualizer *oldVis = box->activityVisualizer();
		m_proxy->positionWidget(oldVis->x(), oldVis->y()-m_proxy->moveBarHeight());
		m_proxy->resizeWidget(oldVis->width(), oldVis->height());
		delete oldVis;
		box->setIsActivityVisuEnabled(false);
	}
	else
		m_proxy->positionWidget(box->scenePos().x(), box->scenePos().y() - m_proxy->widget()->height() -m_proxy->moveBarHeight() - 10);

	box->setDisplayedProxy(m_proxy);
	m_shaderMoveBar->setProxy(m_proxy);

	//draw a link between the box and its visu
	LinkVisuToBox *linkVisuToBox = new LinkVisuToBox(m_shaderMoveBar->x()+m_proxy->widget()->width()/2,
	                                        m_shaderMoveBar->y()+m_proxy->widget()->height()/2,
	                                        box->x()+box->bWidth()/2,
	                                        box->y()+box->bHeight()/2);
	m_proxy->setLinkToBox(linkVisuToBox);
	m_shaderMoveBar->setLinkVisuToBox(linkVisuToBox);
	box->setLinkVisuToBox(linkVisuToBox);

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
