#ifndef ACTIVITYVISUALIZERTHERMAL_H
#define ACTIVITYVISUALIZERTHERMAL_H

#include "activityvisualizer.h"
#include "diagrambox.h"

/**
 * @brief The ActivityVisualizerThermal class is an @ActivityVisualizer, it allows to visualize a 2D
 * matrix of activity.
 */
class ActivityVisualizerThermal : public ActivityVisualizer
{
	Q_OBJECT

public:
	explicit ActivityVisualizerThermal(DiagramBox *box);

	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);
	void keyPressEvent(QKeyEvent *evt);

signals:
	void sizeChanged();

private slots:
	void updateThermal(QVector<qreal> *mat);

public slots:
	void onSizeChanged();
};

#endif // ACTIVITYVISUALIZERTHERMAL_H
