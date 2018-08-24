#include "vectorvisualization.h"
#include "helpers.h"

#include <QDebug>

VectorVisualization::VectorVisualization(QWidget *parent, QGraphicsScene *scene, DiagramBox *box) :
    ScalarVisualization(parent, scene, box)
{
	qDebug() << "[VectorVis] created";

	// Get the size of the vector
	// TODO: change between row and col vis
	if (box->rows() == 1)
		m_size = box->cols();
	else if (box->cols() == 1)
		m_size = box->rows();
	else
		informUserAndCrash(tr("Not a vector!"));

	// Populate the bar set
	m_barSets.clear();
	for (unsigned int i = 0; i < m_size; i += 1) {
		QBarSet *s = new QBarSet(QString::number(i));
		*s << 0;
		m_barSets.append(s);
	}


}
