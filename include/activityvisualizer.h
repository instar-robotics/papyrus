#ifndef ACTIVITYVISUALIZER_H
#define ACTIVITYVISUALIZER_H

#include "diagrambox.h"

#include <QGraphicsPixmapItem>
#include <QImage>

class ActivityVisualizer : public QObject, public QGraphicsPixmapItem
{
	Q_OBJECT

public:
	explicit ActivityVisualizer(DiagramBox *box, QGraphicsItem *parent = nullptr);

	DiagramBox *box() const;
	void setBox(DiagramBox *box);

	QImage image() const;
	void setImage(const QImage &image);

	void updateVisu(QList<qreal>* mat);

private:
	DiagramBox *m_box;

	int m_width;
	int m_height;

	QImage m_image;

private slots:
	void updateMatrix(QVector<qreal> *mat);
};

#endif // ACTIVITYVISUALIZER_H
