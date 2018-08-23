#include "vectorvisualization.h"

#include <QDebug>

VectorVisualization::VectorVisualization(QWidget *parent, QGraphicsScene *scene, DiagramBox *box) :
    DataVisualization(parent, scene, box)
{
	qDebug() << "[VectorVis] created";
	// We populate the available types of visualization for Scalar in the menu
	m_typeMenu->addAction(tr("Bar"), this, SLOT(switchToBar()));
	m_typeMenu->addAction(tr("Graph"), this, SLOT(switchToGraph()));
	m_typeMenu->addAction(tr("Percent"), this, SLOT(switchToPercent()));
}

void VectorVisualization::switchToBar()
{
	qDebug() << "Switched to Bar";
}

void VectorVisualization::switchToGraph()
{
	qDebug() << "Switched to Graph";
}

void VectorVisualization::switchToPercent()
{
	qDebug() << "Switched to Percent";
}
