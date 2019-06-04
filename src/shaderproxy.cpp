#include "shaderproxy.h"

ShaderProxy::ShaderProxy(ShaderWidget *widget, QGraphicsRectItem *moveBar):
    m_widget(widget),
    m_moveBar(moveBar)
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
	setParentItem(nullptr);
	delete m_moveBar;
	delete m_widget;
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

QGraphicsRectItem *ShaderProxy::moveBar() const
{
	return m_moveBar;
}
