#include "activityvisualizer.h"

#include <QDebug>
#include <QCursor>
#include <QGraphicsScene>

// I know there is potentially a segfault here if box == nullptr. But I did not want to use a
// pointer for the QImage. Potential solution is to manually include width and height as params
ActivityVisualizer::ActivityVisualizer(DiagramBox *box)
//    : QGraphicsPixmapItem(box),
    : QGraphicsPixmapItem(nullptr),
      m_box(box),
      m_width(300),
      m_height(101),
      m_cols(box->cols()),
      m_rows(box->rows()),
      m_image(QImage(box->cols(), 101, QImage::Format_RGB32)),
//      m_image2(QImage(box->cols() + m_scaleMargin, 100 + m_nameMargin, QImage::Format_RGB32)),
//      m_doubleBufferFlag(false),
      m_resizeType(NO_RESIZE),
      m_visuTitle(this),
      m_range(1.0),
      m_minWidth(100),
      m_minHeight(100),
      m_activityFetcher(nullptr),
      m_linkToBox(nullptr)
{
	m_box->setActivityVisualizer(this);
	m_box->setIsActivityVisuEnabled(true);

	// Fill background with white
	m_image.fill(qRgb(255, 255, 255));

	// Position the visualizer slightly above its associated box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

	setFlag(QGraphicsItem::ItemIsMovable, true);
	setFlag(QGraphicsItem::ItemIsSelectable, true);
	setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);

	// Add the visualizer to the box's scene
	if (m_box->scene() == nullptr)
		qWarning() << "ActivityVisualier cannot be added to it's box's scene as it has none!";
	else
		m_box->scene()->addItem(this);

	// Delete itself when the box is is attached to is deleted.
	/* NOTE: I don't understand how this doesn't segfault when both the box and the visualizer are
	 * in the scene. As they are in the scene, the scene should take care of deleting these items.
	 * So there's a possibility it can destroy the box first, then this signal/slot makes it delete
	 * itself, and then the scene tries to delete this again...
	 * I can't reproduce this must it seems magical to me :/
	 */
	connect(m_box, SIGNAL(boxDestroyed()), this, SLOT(onBoxDestroyed()));
}

ActivityVisualizer::~ActivityVisualizer()
{
	if (m_activityFetcher != nullptr) {
		m_activityFetcher->setShouldQuit(true);
		m_activityFetcher->wait(1000);
		delete m_activityFetcher;
	}

	if (m_box != nullptr) {
		m_box->setActivityVisualizer(nullptr);
		m_box->setIsActivityVisuEnabled(false);
	}

	if(m_linkToBox != nullptr)
	{
		delete m_linkToBox;
	}
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
	qreal margin = 5;

	if (mouseX >= m_width - margin && mouseY >= m_height - margin)
		setCursor(QCursor(Qt::SizeFDiagCursor));
	else if (mouseX >= m_width - margin)
		setCursor(QCursor(Qt::SizeHorCursor));
	else if (mouseY >= m_height - margin)
		setCursor(QCursor(Qt::SizeVerCursor));
	else
		setCursor(QCursor(Qt::ArrowCursor));

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
	qreal margin = 5;

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
	qreal newWidth, newHeight;

	switch (m_resizeType) {
		case RESIZE_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= m_minWidth ? newWidth : m_minWidth;
		break;

		case RESIZE_BOTTOM:
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= m_minHeight ? newHeight : m_minHeight;
		break;

		case RESIZE_BOTTOM_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= m_minWidth ? newWidth : m_minWidth;
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= m_minHeight ? newHeight : m_minHeight;
		break;

		default:
			;
	}
	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));
	updateLinkToBox(scenePos());

	QGraphicsPixmapItem::mouseMoveEvent(evt);
}

QVariant ActivityVisualizer::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
	if ((change == QGraphicsItem::ItemPositionChange || change == QGraphicsItem::ItemScenePositionHasChanged) && scene()) {
		// Get coordinate of the target new position
		QPointF newPos = value.toPointF();
		updateLinkToBox(newPos);
		return newPos;
	}
	return QGraphicsItem::itemChange(change, value);
}

void ActivityVisualizer::updatePixmap()
{
	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));
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

ActivityFetcher *ActivityVisualizer::activityFetcher() const
{
	return m_activityFetcher;
}

void ActivityVisualizer::setActivityFetcher(ActivityFetcher *activityFetcher)
{
	m_activityFetcher = activityFetcher;
}

int ActivityVisualizer::width() const
{
	return m_width;
}

void ActivityVisualizer::setWidth(int width)
{
	m_width = width;
}

int ActivityVisualizer::height() const
{
	return m_height;
}

void ActivityVisualizer::setHeight(int height)
{
	m_height = height;
}

void ActivityVisualizer::setLinkToBox(LinkVisuToBox *linkToBox)
{
	m_linkToBox = linkToBox;
}

void ActivityVisualizer::updateLinkToBox(QPointF newPos)
{
	// When the visu is moved, the link to its diagram box is moved too
	if(m_linkToBox != nullptr)
		m_linkToBox->centerVisuMoved(newPos.x()+width()/2.0, newPos.y()+height()/2.0);
}


void ActivityVisualizer::onBoxDestroyed()
{
	delete this;
}
