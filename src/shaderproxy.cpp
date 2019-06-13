#include "shaderproxy.h"

ShaderProxy::ShaderProxy(ShaderWidget *widget, ShaderMoveBar *moveBar, DiagramBox * box):
    m_widget(widget),
    m_moveBar(moveBar),
    m_box(box)
{

	setWidget(m_widget);
	//m_widget->resizeGL(200,200);
	m_moveBar->setRect(0, 0, m_widget->width(), 20);
	m_moveBar->setPen(QPen(Qt::black));
	m_moveBar->setBrush(QBrush(Qt::black));
	setPos(0, m_moveBar->rect().height());
	setParentItem(m_moveBar);

	m_moveBar->setFlag(QGraphicsItem::ItemIsMovable, true);
	m_moveBar->setFlag(QGraphicsItem::ItemIsSelectable, true);

	connect(m_widget, SIGNAL(repaint()), this, SLOT(updateProxy()));
}

ShaderProxy::~ShaderProxy()
{
	if (m_activityFetcher != nullptr) {
		m_activityFetcher->setShouldQuit(true);
		m_activityFetcher->wait(1000);
		delete m_activityFetcher;
	}
	if (m_box != nullptr) {
		m_box->setDisplayedProxy(nullptr);
		m_box->setActivityVisualizer(nullptr);
		m_box->setIsActivityVisuEnabled(false);
	}

	setParentItem(nullptr);
	delete m_moveBar;
}

void ShaderProxy::updateProxy()
{
	update();
}

void ShaderProxy::updateValues(QVector<qreal>* values)
{
	m_widget->updateValues(values);
	//delete values;
}
void ShaderProxy::setActivityFetcher(ActivityFetcher *activityFetcher)
{
	m_activityFetcher = activityFetcher;
}

ShaderMoveBar *ShaderProxy::moveBar() const
{
	return m_moveBar;
}

void ShaderProxy::wheelEvent(QGraphicsSceneWheelEvent *event)
{
	m_widget->wheelTurned(event->delta());
}

