#include "activityvisualizerbars.h"
#include "helpers.h"

#include <QDebug>

// I know there is potentially a segfault here if box == nullptr. But I did not want to use a
// pointer for the QImage. Potential solution is to manually include width and height as params
ActivityVisualizerBars::ActivityVisualizerBars(DiagramBox *box, QGraphicsItem *parent)
    : ActivityVisualizer(box, parent),
      m_scaleMargin(10),
//      m_image2(QImage(box->cols() + m_scaleMargin, 100 + m_nameMargin, QImage::Format_RGB32)),
//      m_doubleBufferFlag(false),
      m_hLine(this),
      m_vLine(this),
      m_nbTicks(5), // keep it odd to have 0 displayed
      m_range(1.0)
{
	// Define orientation based on the box's dimensions
	if (m_box->outputType() == SCALAR) {
		m_barsOrientation = HORIZONTAL;
	} else if (m_box->outputType() == MATRIX) {
		if (m_box->rows() == 1)
			m_barsOrientation = HORIZONTAL;
		else if (m_box->cols() == 1)
			m_barsOrientation = VERTICAL;
		else {
			informUserAndCrash(tr("Bars visualization is only valid for scalar, (1,1) matrices and vectors."));
		}
	} else {
		informUserAndCrash(tr("Activity visualization is only supported for SCALAR and MATRIX."));
	}

	// Set size based on orientation
	if (m_barsOrientation == HORIZONTAL) {
		m_width = 300;
		m_height = 101; // Keep odd to be able to center axis line

		// Convenience: when a very small numbers of neurons, make default window smaller
		if (m_box->cols() < 15)
			m_width = 120;

		// Create the image: it has fixed height (just convenience display) but its width is the number of columns
		m_image = QImage(m_box->cols(), m_height, QImage::Format_RGB32);
	} else {
		m_width = 101; // Keep odd to be able to center axis line
		m_height = 300;

		// Convenience: when a very small numbers of neurons, make default window smaller
		if (m_box->rows() < 15)
			m_height = 120;

		// Create the image: it has fixed width (just convenience display) but its height is the number of rows
		m_image = QImage(m_width, m_box->rows(), QImage::Format_RGB32);
	}

	// Fill background with white
	m_image.fill(qRgb(255, 255, 255));
//	m_image2.fill(qRgb(239, 239, 239));

	// Position the visualizer slighty above its associated box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

	m_painter.begin(&m_image);
//	m_painter2.begin(&m_image2);

	// Set the pixmap from the image
//	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));
	updatePixmap();

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
	onSizeChanged();

	connect(this, SIGNAL(sizeChanged()), this, SLOT(onSizeChanged()));
}

ActivityVisualizerBars::~ActivityVisualizerBars()
{
	for (int i = 0; i < m_nbTicks; i += 1) {
		delete m_ticks.at(i);
		delete m_labels.at(i);
	}
}

/**
 * @brief ActivityVisualizerBars::mouseMoveEvent is used to resize the chart when it is in resizing mode.
 * @param evt
 */
void ActivityVisualizerBars::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
	// Define some minimum dimensions so we can't make charts too small
	qreal newWidth, newHeight;

	switch (m_resizeType) {
		case RESIZE_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= m_minWidth ? newWidth : m_minWidth;
			emit sizeChanged();
		break;

		case RESIZE_BOTTOM:
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= m_minHeight ? newHeight : m_minHeight;
			emit sizeChanged();
		break;

		case RESIZE_BOTTOM_RIGHT:
			newWidth = evt->scenePos().x() - scenePos().x();
			m_width = newWidth >= m_minWidth ? newWidth : m_minWidth;
			newHeight = evt->scenePos().y() - scenePos().y();
			m_height = newHeight >= m_minHeight ? newHeight : m_minHeight;
			emit sizeChanged();
		break;

		default:
			;
	}

//	setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));
	updatePixmap();

	QGraphicsPixmapItem::mouseMoveEvent(evt);
}

/**
 * @brief ActivityVisualizerBars::keyPressEvent is used to match on a press to ESCAPE, to close
 * this visualizer.
 * @param evt
 */
void ActivityVisualizerBars::keyPressEvent(QKeyEvent *evt)
{
	int key = evt->key();

	if (key == Qt::Key_Escape || key == Qt::Key_Delete)
		delete this;

	QGraphicsPixmapItem::keyPressEvent(evt);
}

// TODO: can we implement double-buffering using two QImages?
// TODO: we can use QPaint to paint directly on the pixmap, let's try painting directly the columns
// of pixels instead
// TODO: support higher values that 1 and update scale dynamically
void ActivityVisualizerBars::updateBars(QVector<qreal> *mat)
{
	if (mat == nullptr) {
		qWarning() << "[updateBars] got no data";
		return;
	}

	// Don't do anything is we're hidden (no need to take up resources)
	if (!isVisible()) {
		delete mat;
		return;
	}

	int rows = m_box->rows();
	int cols = m_box->cols();

	if (mat->size() == rows * cols) {
		QColor blue(51, 153, 255);
		QColor red(246, 2, 2);

		// Define pointer accordingly to double buffering
		//	QImage *image = m_doubleBufferFlag ? &m_image2 : &m_image;
		//	QPainter *painter = m_doubleBufferFlag ? &m_painter2 : &m_painter;
		QImage *image = &m_image;
		QPainter *painter = &m_painter;

		// Swap the buffers
		//	m_doubleBufferFlag = !m_doubleBufferFlag;

		// Erase previous display
		//	m_image.fill(qRgb(239, 239, 239));
		//	painter->fillRect(QRect(0, 0, cols, 100), QColor(239, 239, 239));
		if (m_barsOrientation == HORIZONTAL) {
			painter->fillRect(QRect(0, 0, cols, m_height), QColor(255, 255, 255));

			// Update pixels in the QImage
			for (int i = 0; i < cols; i += 1) {
				// Make sure the value is comprised between [-1; +1]
				double capped = mat->at(i);
				capped = capped > 1.0 ? 1.0 : (capped < -1.0 ? -1.0 : capped);

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
		} else {
			painter->fillRect(QRect(0, 0, m_width, rows), QColor(255, 255, 255));

			// Update pixels in the QImage
			for (int j = 0; j < rows; j += 1) {
				// Make sure the value is comprised between [-1; +1]
				double capped = mat->at(j);
				capped = capped > 1.0 ? 1.0 : (capped < -1.0 ? -1.0 : capped);

				if (capped >= 0) {
					for (int i = 0; i < capped * 50; i += 1) {
						image->setPixel(50-i, j, red.rgb());
					}
				} else {
					for (int i = 0; i < -capped * 50; i += 1) {
						image->setPixel(50+i, j, blue.rgb());
					}
				}
			}
		}

		// Update pixmap from image
//		setPixmap(QPixmap::fromImage(*image).scaled(m_width, m_height));
		updatePixmap();
	} else {
		qWarning() << "Invalid number of data to update bars: "
		           << mat->size() << "data points for" << rows << "x" << cols;
	}

	// Don't forget to delete matrix pointer to avoid memory leak
	delete mat;
}

/**
 * @brief ActivityVisualizerBars::updateLines is called when the graph window is resized.
 */
void ActivityVisualizerBars::onSizeChanged()
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
