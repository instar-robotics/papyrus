#ifndef ACTIVITYVISUALIZER_H
#define ACTIVITYVISUALIZER_H

#include "diagrambox.h"

#include <QGraphicsPixmapItem>
#include <QImage>
#include <QGraphicsSceneHoverEvent>
#include <QPainter>

/**
 * @brief The ActivityVisualizer class is a QImage (inside a QGraphicsPixmapItem) placed on the
 * scene to visualize the output of a box's activity, live.
 */

class ActivityVisualizer : public QObject, public QGraphicsPixmapItem
{
	Q_OBJECT

public:
	explicit ActivityVisualizer(DiagramBox *box, QGraphicsItem *parent = nullptr);
	~ActivityVisualizer();

	void hoverMoveEvent(QGraphicsSceneHoverEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

	DiagramBox *box() const;
	void setBox(DiagramBox *box);

	QImage image() const;
	void setImage(const QImage &image);

	void updateVisu(QList<qreal>* mat);

private:
	DiagramBox *m_box;

	int m_width;   // The width of the display (in px)
	int m_height;  // The height of the display (in px)
	int m_cols;    // The number of neurons horizontally
	int m_rows;    // The number of neurons vertically

	int m_scaleMargin; // Left / top margin used to draw scale on the graph

	QImage m_image;
//	QImage m_image2;
//	bool m_doubleBufferFlag;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing
	QPainter m_painter;
//	QPainter m_painter2;

	QGraphicsLineItem m_hLine;
	QGraphicsLineItem m_vLine;
	QGraphicsTextItem m_visuTitle;

	int m_nbTicks;
	QList<QGraphicsLineItem *> m_ticks;  // List of ticks on the axis

	qreal m_range;
	QList<QGraphicsTextItem *> m_labels; // List of labels for the ticks

signals:
//	void sizeChanged(QSize newSize);
	void sizeChanged();

private slots:
	void updateMatrix(QVector<qreal> *mat);
	void updateAxes();
};

#endif // ACTIVITYVISUALIZER_H
