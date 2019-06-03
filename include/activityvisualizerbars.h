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
	void keyPressEvent(QKeyEvent *evt);
	void wheelEvent(QGraphicsSceneWheelEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);

private:
	int m_scaleMargin; // Left / top margin used to draw scale on the graph

	QGraphicsLineItem m_hLine;
	QGraphicsLineItem m_vLine;

	int m_nbTicks;
	QList<QGraphicsLineItem *> m_ticks;  // List of ticks on the axis

	qreal m_range;
	QList<QGraphicsTextItem *> m_labels; // List of labels for the ticks

	BarsOrientation m_barsOrientation;

	QVector<qreal> *m_lastMat; // Keep the last used matrix to be able to redraw when adjusting size

	QGraphicsTextItem m_scalarValue; // Used to display the value of a single scalar when type is SCALAR

	QGraphicsLineItem m_beginTick;   // The tick that marks the first neuron
	QGraphicsLineItem m_middleTick;  // The tick that marks the middle of the vector
	QGraphicsLineItem m_endTick;     // The tick that marks the last neuron

signals:
	void sizeChanged();

private slots:
	void updateBars(QVector<qreal> *mat);
	void onBoxDeleted();

public slots:
	void onSizeChanged();
};

#endif // ACTIVITYVISUALIZERBARS_H
