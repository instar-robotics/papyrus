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
      m_scaleMargin(10),
//      m_image(QImage(box->cols() + m_scaleMargin, 100 + m_nameMargin, QImage::Format_RGB32)),
      m_image(QImage(box->cols(), 101, QImage::Format_RGB32)),
//      m_image2(QImage(box->cols() + m_scaleMargin, 100 + m_nameMargin, QImage::Format_RGB32)),
//      m_doubleBufferFlag(false),
      m_resizeType(NO_RESIZE),
      m_hLine(this),
      m_vLine(this),
      m_visuTitle(this),
      m_nbTicks(5), // keep it odd to have 0 displayed
      m_range(1.0)
{
	// Fill background with white
//	m_image.fill(qRgb(239, 239, 239));
	m_image.fill(qRgb(255, 255, 255));
//	m_image2.fill(qRgb(239, 239, 239));

	// Position the visualizer slighty above its associated box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

	setFlag(QGraphicsItem::ItemSendsGeometryChanges);
	setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
	setAcceptHoverEvents(true);

	m_painter.begin(&m_image);
//	m_painter2.begin(&m_image2);

	// Set the pixmap from the image
	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));

	// Create a text item to display the function's name
	m_visuTitle.setHtml(QString("<center>%1</center>").arg(m_box->name()));
	if (!m_box->title().isEmpty())
		m_visuTitle.setHtml(QString("<center>%1</center>").arg(m_box->title()));

	QFont titleFont = m_visuTitle.font();
	titleFont.setPointSize(titleFont.pointSize() - 4);
	m_visuTitle.setFont(titleFont);

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

	connect(this, SIGNAL(sizeChanged()), this, SLOT(updateAxes()));
}

ActivityVisualizer::~ActivityVisualizer()
{
	for (int i = 0; i < m_nbTicks; i += 1) {
		delete m_ticks.at(i);
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
	qreal minWidth = 100;
	qreal minHeight = 100;
	qreal newWidth, newHeight;

	switch (m_resizeType) {
		case RESIZE_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= minWidth ? newWidth : minWidth;
			emit sizeChanged();
		break;

		case RESIZE_BOTTOM:
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= minHeight ? newHeight : minHeight;
			emit sizeChanged();
		break;

		case RESIZE_BOTTOM_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= minWidth ? newWidth : minWidth;
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= minHeight ? newHeight : minHeight;
			emit sizeChanged();
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

	QColor blue(51, 153, 255);
	QColor red(246, 2, 2);

	// Define pointer accordingly to double buffering
//	QImage *image = m_doubleBufferFlag ? &m_image2 : &m_image;
//	QPainter *painter = m_doubleBufferFlag ? &m_painter2 : &m_painter;
	QImage *image = &m_image;
	QPainter *painter = &m_painter;

	// Switch double buffering
//	m_doubleBufferFlag = !m_doubleBufferFlag;

	// Erase previous display
//	m_image.fill(qRgb(239, 239, 239));
//	painter->fillRect(QRect(0, 0, cols, 100), QColor(239, 239, 239));
	painter->fillRect(QRect(0, 0, cols, m_height), QColor(255, 255, 255));

	// Update pixels in the QImage
	for (int i = 0; i < cols; i += 1) {
		// Make sure the value is comprised between [-1; +1]
		double capped = mat->at(i);
		capped = capped > 1.0 ? 1.0 : (capped < -1.0 ? -1.0 : capped);

		// Normalize the value between [0; 1] for multiplication
//		double normalizedValue = (capped - 1.0) / (-2.0);

		if (capped >= 0) {
			for (int j = 0; j < capped * 50; j += 1) {
				image->setPixel(i, 50-j, blue.rgb());
			}
		} else {
			for (int j = 0; j < -capped * 50; j += 1) {
				image->setPixel(i, 50+j, red.rgb());
			}
		}
	}

	// Update pixmap from image
	setPixmap(QPixmap::fromImage(*image).scaled(m_width, m_height));

	// Don't forget to delete matrix pointer to avoid memory leak
	delete mat;
}

/**
 * @brief ActivityVisualizer::updateLines is called when the graph window is resized.
 */
void ActivityVisualizer::updateAxes()
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
