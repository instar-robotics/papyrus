#include "activityvisualizer.h"

#include <QDebug>

// I know there is potentially a segfault here if box == nullptr. But I did not want to use a
// pointer for the QImage. Potential solution is to manually include width and height as params
ActivityVisualizer::ActivityVisualizer(DiagramBox *box, QGraphicsItem *parent)
    : QGraphicsPixmapItem(parent),
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
      m_activityFetcher(nullptr)
{
	// Fill background with white
	m_image.fill(qRgb(255, 255, 255));

	// Position the visualizer slightly above its associated box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

//	setFlag(QGraphicsItem::ItemSendsGeometryChanges);
//	setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
	setFlag(QGraphicsItem::ItemIsFocusable);
	setAcceptHoverEvents(true);

	// By default, the font is too big, reduce it
//	QFont titleFont = m_visuTitle.font();
//	titleFont.setPointSize(titleFont.pointSize() - 4);
//	m_visuTitle.setFont(titleFont);

	/*
	m_painter.begin(&m_image);
//	m_painter2.begin(&m_image2);

	// Set the pixmap from the image
	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));

	// Create a text item to display the function's name
	m_visuTitle.setHtml(QString("<center>%1</center>").arg(m_box->name()));
	if (!m_box->title().isEmpty())
		m_visuTitle.setHtml(QString("<center>%1</center>").arg(m_box->title()));

	// Create the ticks & labels
	for (int i = 0; i < m_nbTicks; i += 1) {
		QGraphicsLineItem *tick = new QGraphicsLineItem(this);
		m_ticks << tick;

		QGraphicsTextItem *label = new QGraphicsTextItem(this);
		QFont labelFont = label->font();
		labelFont.setPointSizeF(labelFont.pointSizeF() - 4);
		label->setFont(labelFont);
		m_labels << label;
	}

	// Create the horizontal and vertical lines (axes)
	updateAxes();

	connect(this, SIGNAL(sizeChanged()), this, SLOT(onSizeChanged()));
	//*/
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
//			emit sizeChanged();
		break;

		case RESIZE_BOTTOM:
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= m_minHeight ? newHeight : m_minHeight;
//			emit sizeChanged();
		break;

		case RESIZE_BOTTOM_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= m_minWidth ? newWidth : m_minWidth;
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= m_minHeight ? newHeight : m_minHeight;
//			emit sizeChanged();
		break;

		default:
			;
	}

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

ActivityFetcher *ActivityVisualizer::activityFetcher() const
{
	return m_activityFetcher;
}

void ActivityVisualizer::setActivityFetcher(ActivityFetcher *activityFetcher)
{
	m_activityFetcher = activityFetcher;
}

/**
 * @brief ActivityVisualizer::updateLines is called when the graph window is resized.
 */
/*
void ActivityVisualizer::onSizeChanged()
{
	// Create a horizontal line (axis)
	m_hLine.setLine(-m_scaleMargin, m_height / 2, m_width + m_scaleMargin, m_height / 2);

	// Create a vertical line (axis)
	m_vLine.setLine(-m_scaleMargin, -m_scaleMargin, -m_scaleMargin, m_height + m_scaleMargin);

	// The only way to center text is to use setHtml() AND set the TextWidth
	m_visuTitle.setTextWidth(m_width);

	m_visuTitle.setPos(0, m_height);

	qreal dist = m_height / (m_nbTicks - 1);
	qreal tickDiff = 2 * m_range / (m_nbTicks - 1);
	for (int i = 0; i < m_nbTicks; i += 1) {
		m_ticks.at(i)->setLine(-m_scaleMargin,
							   i * dist,
							   -m_scaleMargin / 2,
							   i * dist);

		m_labels.at(i)->setPlainText(QString::number(m_range - i * tickDiff));
		QRectF r = m_labels.at(i)->boundingRect();
		m_labels.at(i)->setPos(-m_scaleMargin - r.width(), // right align
									   i * dist - r.height() / 2); // middle align
	}
}
	//*/
