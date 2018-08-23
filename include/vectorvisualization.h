#ifndef VECTORVISUALIZATION_H
#define VECTORVISUALIZATION_H

#include "datavisualization.h"

class VectorVisualization : public DataVisualization
{
	Q_OBJECT
public:
	VectorVisualization(QWidget *parent = nullptr, QGraphicsScene *scene = nullptr, DiagramBox *box = nullptr);

private slots:
	void switchToBar();
	void switchToGraph();
	void switchToPercent();
};

#endif // VECTORVISUALIZATION_H
