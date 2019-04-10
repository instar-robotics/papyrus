#include "activityvisualizer.h"

#include <QDebug>

// I know there is potentially a segfault here if box == nullptr. But I did not want to use a
// pointer for the QImage. Potential solution is to manually include width and height as params
ActivityVisualizer::ActivityVisualizer(DiagramBox *box, QGraphicsItem *parent)
    : QGraphicsPixmapItem(parent),
      m_box(box),
      m_width(box->cols()),
//      m_height(box->rows()),
      m_height(100), // temporary
      m_image(QImage(box->cols(), 100, QImage::Format_RGB32))
{
	qDebug() << "ActivityVisualizer created with size" << m_image.size() << "(" << m_width << "x" << m_height << ") and SCALED";

	// Fill background with a light gray
	m_image.fill(qRgb(239, 239, 239));

	// Set the pixmap from the image
	setPixmap(QPixmap::fromImage(m_image).scaled(300, 100));

	// Position the visualizer slight above its associate box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

	setFlag(QGraphicsItem::ItemSendsGeometryChanges);
	setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
	setAcceptHoverEvents(true);
}

DiagramBox *ActivityVisualizer::box() const
{
	return m_box;
}

void ActivityVisualizer::setBox(DiagramBox *box)
{
	m_box = box;
}

QImage ActivityVisualizer::image() const
{
	return m_image;
}

void ActivityVisualizer::setImage(const QImage &image)
{
	m_image = image;
}

void ActivityVisualizer::updateVisu(QList<qreal> *mat)
{
	/*
	int rows = m_box->rows();
	int cols = m_box->cols();

	QColor color(51, 153, 255);

	// Update pixels in the QImage
	for (int i = 0; i < cols; i += 1) {
		// Make sure the value is comprised between [-1; +1]
		double capped = mat->at(i);
		capped = capped > 1.0 ? 1.0 : (capped < -1.0 ? -1.0 : capped);

		// Normalize the value between [0; 1] for multiplication
		//		double normalizedValue = (capped - 1.0) / (-2.0);

		if (capped >= 0) {
			for (int j = 0; j < capped * 50; j += 1) {
				m_image.setPixel(i, 50-j, color.rgb());
			}
		} else {
			for (int j = 0; j < -capped * 50; j += 1) {
				m_image.setPixel(i, 50+j, color.rgb());
			}
		}
	}
	//*/

	// Update pixmap from image

	// CANNOT be called from another thread.
	// So goes either SIGNALS / SLOTS or QCoreApplication::postEvent()
	setPixmap(QPixmap::fromImage(m_image));
}

// TODO: can we implement double-buffering using two QImages?
// TODO: we can use QPaint to paint directly on the pixmal, let's try painting directly the columns
// of pixels instead
void ActivityVisualizer::updateMatrix(QVector<qreal> *mat)
{
	int rows = m_box->rows();
	int cols = m_box->cols();

	QColor color(51, 153, 255);

	// Erase previous display
	m_image.fill(qRgb(239, 239, 239));

	// Update pixels in the QImage
	for (int i = 0; i < cols; i += 1) {
		// Make sure the value is comprised between [-1; +1]
		double capped = mat->at(i);
		capped = capped > 1.0 ? 1.0 : (capped < -1.0 ? -1.0 : capped);

		// Normalize the value between [0; 1] for multiplication
		//		double normalizedValue = (capped - 1.0) / (-2.0);

		if (capped >= 0) {
			for (int j = 0; j < capped * 50; j += 1) {
				m_image.setPixel(i, 50-j, color.rgb());
			}
		} else {
			for (int j = 0; j < -capped * 50; j += 1) {
				m_image.setPixel(i, 50+j, color.rgb());
			}
		}
	}

	// Update pixmap from image
	setPixmap(QPixmap::fromImage(m_image).scaled(300, 100));

	// DONT FORGET TO DELETE THE MATRIX POINTER
	delete mat;
}
