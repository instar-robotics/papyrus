#ifndef ACTIVITYVISUALIZER_H
#define ACTIVITYVISUALIZER_H

#include "diagrambox.h"

#include <QGraphicsPixmapItem>
#include <QImage>
#include <QGraphicsSceneHoverEvent>

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

	void updateVisu(QList<qreal>* mat);

private:
	DiagramBox *m_box;

	int m_width;   // The width of the display (in px)
	int m_height;  // The height of the display (in px)
	int m_cols;    // The number of neurons horizontally
	int m_rows;    // The number of neurons vertically

	QImage m_image;

	ResizeType m_resizeType; // Type of resizing operation we are currently performing

private slots:
	void updateMatrix(QVector<qreal> *mat);
};

#endif // ACTIVITYVISUALIZER_H
