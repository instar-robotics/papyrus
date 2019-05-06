#ifndef ACTIVITYVISUALIZERBARS_H
#define ACTIVITYVISUALIZERBARS_H

#include "diagrambox.h"
#include "activityvisualizer.h"

#include <QGraphicsPixmapItem>
#include <QImage>
#include <QGraphicsSceneHoverEvent>
#include <QPainter>

enum BarsOrientation {
	HORIZONTAL, // For scalar, (1,1)-matrix and row vectors
	VERTICAL    // For column vectors
};

/**
 * @brief The ActivityVisualizerBars is an @ActivityVisualizer to view scalar, (1,1)-matrix and
 * vectors. It is automatically set to horizontal or vertical orientation based on the dimensions
 * of the matrix.
 */

class ActivityVisualizerBars : public ActivityVisualizer
{
	Q_OBJECT

public:
	explicit ActivityVisualizerBars(DiagramBox *box, QGraphicsItem *parent = nullptr);
	~ActivityVisualizerBars();

	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

private:
	int m_scaleMargin; // Left / top margin used to draw scale on the graph

	QGraphicsLineItem m_hLine;
	QGraphicsLineItem m_vLine;

	int m_nbTicks;
	QList<QGraphicsLineItem *> m_ticks;  // List of ticks on the axis

	qreal m_range;
	QList<QGraphicsTextItem *> m_labels; // List of labels for the ticks

	BarsOrientation m_barsOrientation;

signals:
	void sizeChanged();

private slots:
	void updateBars(QVector<qreal> *mat);
	void onSizeChanged();
};

#endif // ACTIVITYVISUALIZERBARS_H
