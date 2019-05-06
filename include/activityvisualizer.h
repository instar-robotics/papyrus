#ifndef ACTIVITYVISUALIZER_H
#define ACTIVITYVISUALIZER_H

#include "diagrambox.h"
#include "activityfetcher.h"

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

	void hoverMoveEvent(QGraphicsSceneHoverEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

	DiagramBox *box() const;
	void setBox(DiagramBox *box);

	QImage image() const;
	void setImage(const QImage &image);

	ActivityFetcher *activityFetcher() const;
	void setActivityFetcher(ActivityFetcher *activityFetcher);

protected:
	DiagramBox *m_box;

	int m_width;   // The width of the display (in px)
	int m_height;  // The height of the display (in px)
	int m_cols;    // The number of neurons horizontally
	int m_rows;    // The number of neurons vertically

	QImage m_image;
//	QImage m_image2;
//	bool m_doubleBufferFlag;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing
	QPainter m_painter;
//	QPainter m_painter2;

	QGraphicsTextItem m_visuTitle;

	qreal m_range;

	qreal m_minWidth;
	qreal m_minHeight;

	ActivityFetcher *m_activityFetcher;

signals:
//	void sizeChanged();

protected slots:
//	void onSizeChanged();
};

#endif // ACTIVITYVISUALIZER_H
