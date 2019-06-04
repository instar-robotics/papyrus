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
	~ActivityVisualizer();

	void hoverMoveEvent(QGraphicsSceneHoverEvent *evt);
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);

	void updatePixmap();

	DiagramBox *box() const;
	void setBox(DiagramBox *box);

	QImage image() const;
	void setImage(const QImage &image);

	ActivityFetcher *activityFetcher() const;
	void setActivityFetcher(ActivityFetcher *activityFetcher);

	int width() const;
	void setWidth(int width);

	int height() const;
	void setHeight(int height);

protected:
	DiagramBox *m_box;

	int m_width;   // The width of the display (in px)
	int m_height;  // The height of the display (in px)
	int m_cols;    // The number of neurons horizontally
	int m_rows;    // The number of neurons vertically

	QImage m_image;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing
	QPainter m_painter;

	QGraphicsTextItem m_visuTitle;

	qreal m_range;

	qreal m_minWidth;
	qreal m_minHeight;

	ActivityFetcher *m_activityFetcher;
};

#endif // ACTIVITYVISUALIZER_H
