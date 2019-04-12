#include "activityvisualizer.h"

#include <QDebug>

// I know there is potentially a segfault here if box == nullptr. But I did not want to use a
// pointer for the QImage. Potential solution is to manually include width and height as params
ActivityVisualizer::ActivityVisualizer(DiagramBox *box, QGraphicsItem *parent)
    : QGraphicsPixmapItem(parent),
      m_box(box),
      m_width(300),
      m_height(100),
      m_cols(box->cols()),
      m_rows(box->rows()),
      m_image(QImage(box->cols(), 100, QImage::Format_RGB32)),
      m_resizeType(NO_RESIZE)
{
	qDebug() << "ActivityVisualizer created with size" << m_image.size() << "(" << m_width << "x" << m_height << ") and SCALED";

	// Fill background with a light gray
	m_image.fill(qRgb(239, 239, 239));

	// Set the pixmap from the image
	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, 100));

	// Position the visualizer slighty above its associated box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

	setFlag(QGraphicsItem::ItemSendsGeometryChanges);
	setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
	setAcceptHoverEvents(true);
}

/**
 * @brief ActivityVisualizer::hoverMoveEvent is used to change the cursor's shape based on its
 * position inside the graph, to hint that it can be resized.
 * @param evt
 */
void ActivityVisualizer::hoverMoveEvent(QGraphicsSceneHoverEvent *evt)
{
	qreal mouseX = evt->pos().x();
	qreal mouseY = evt->pos().y();
	qreal margin = 20;

	if (mouseX >= m_width - margin && mouseY >= m_height - margin)
		setCursor(Qt::SizeFDiagCursor);
	else if (mouseX >= m_width - margin)
		setCursor(Qt::SizeHorCursor);
	else if (mouseY >= m_height - margin)
		setCursor(Qt::SizeVerCursor);
	else
		setCursor(Qt::ArrowCursor);

	QGraphicsPixmapItem::hoverMoveEvent(evt);
}

/**
 * @brief ActivityVisualizer::mousePressEvent is used to temporarily disable the graph's being movable
 * and switch the graph into the correct resizing move, based on where the use clicked its mouse.
 * @param evt
 */
void ActivityVisualizer::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
	qreal mouseX = evt->pos().x();
	qreal mouseY = evt->pos().y();
	qreal margin = 20;

	if (mouseX >= m_width - margin && mouseY >= m_height - margin)
		m_resizeType = RESIZE_BOTTOM_RIGHT;
	else if (mouseX >= m_width - margin)
		m_resizeType = RESIZE_RIGHT;
	else if (mouseY >= m_height - margin)
		m_resizeType = RESIZE_BOTTOM;
	else
		m_resizeType = NO_RESIZE;

	if (m_resizeType == NO_RESIZE)
		setFlag(QGraphicsItem::ItemIsMovable, true);
	else
		setFlag(QGraphicsItem::ItemIsMovable, false);

	QGraphicsPixmapItem::mousePressEvent(evt);
}

/**
 * @brief ActivityVisualizer::mouseReleaseEvent is used to switch off resizing mode when releasing the
 * mouse button
 * @param evt
 */
void ActivityVisualizer::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
	m_resizeType = NO_RESIZE;

	QGraphicsPixmapItem::mouseReleaseEvent(evt);
}

/**
 * @brief ActivityVisualizer::mouseMoveEvent is used to resize the chart when it is in resizing mode.
 * @param evt
 */
void ActivityVisualizer::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
	// Define some minimum dimensions so we can't make charts too small
	qreal minWidth = 100;
	qreal minHeight = 100;
	qreal newWidth, newHeight;

	switch (m_resizeType) {
		case RESIZE_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= minWidth ? newWidth : minWidth;
		break;

		case RESIZE_BOTTOM:
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= minHeight ? newHeight : minHeight;
		break;

		case RESIZE_BOTTOM_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= minWidth ? newWidth : minWidth;
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= minHeight ? newHeight : minHeight;
		break;

		default:
			;
	}

//	resize(m_width, m_height);
	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));

	QGraphicsPixmapItem::mouseMoveEvent(evt);
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
	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));

	// DONT FORGET TO DELETE THE MATRIX POINTER
	delete mat;
}
