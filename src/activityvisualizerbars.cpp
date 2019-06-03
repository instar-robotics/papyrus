#include "activityvisualizerbars.h"
#include "helpers.h"

#include <QDebug>

// I know there is potentially a segfault here if box == nullptr. But I did not want to use a
// pointer for the QImage. Potential solution is to manually include width and height as params
ActivityVisualizerBars::ActivityVisualizerBars(DiagramBox *box, QGraphicsItem *parent)
    : ActivityVisualizer(box, parent),
      m_scaleMargin(10),
      m_hLine(this),
      m_vLine(this),
      m_nbTicks(5), // keep it odd to have 0 displayed
      m_range(1.0),
      m_lastMat(nullptr),
      m_scalarValue(this),
      m_beginTick(this),
      m_middleTick(this),
      m_endTick(this)
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
		m_height = 100;//101; // Keep odd to be able to center axis line

		// Convenience: when a very small numbers of neurons, make default window smaller
		if (m_box->cols() < 15)
			m_width = 120;

		// Create the image: it has fixed height (just convenience display) but its width is the number of columns
		m_image = QImage(m_box->cols(), m_height, QImage::Format_RGB32);
	} else {
		m_width = 100; //101; // Keep odd to be able to center axis line
		m_height = 300;

		// Convenience: when a very small numbers of neurons, make default window smaller
		if (m_box->rows() < 15)
			m_height = 120;

		// Create the image: it has fixed width (just convenience display) but its height is the number of rows
		m_image = QImage(m_width, m_box->rows(), QImage::Format_RGB32);
	}

	// Fill background with white
	m_image.fill(qRgb(255, 255, 255));

	// Position the visualizer slighty above its associated box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

	m_painter.begin(&m_image);

	// Set the pixmap from the image
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

	QFont valueFont = m_scalarValue.font();
	valueFont.setPointSize(valueFont.pointSize() - 2);
	m_scalarValue.setFont(valueFont);

	// Create the horizontal and vertical lines (axes)
	onSizeChanged();

	connect(this, SIGNAL(sizeChanged()), this, SLOT(onSizeChanged()));
	connect(m_box, SIGNAL(boxDeleted()), this, SLOT(onBoxDeleted()));
}

ActivityVisualizerBars::~ActivityVisualizerBars()
{
	for (int i = 0; i < m_nbTicks; i += 1) {
		delete m_ticks.at(i);
		delete m_labels.at(i);
	}

	if (m_lastMat != nullptr)
		delete m_lastMat;
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

/**
 * @brief ActivityVisualizerBars::wheelEvent is used to zoom or dezoom the scale, when SHIFT is
 * held and the wheel is scrolled
 * @param evt
 */
void ActivityVisualizerBars::wheelEvent(QGraphicsSceneWheelEvent *evt)
{
	// Only do something if we pressed SHIFT
	if (evt->modifiers() & Qt::ShiftModifier) {
		if (evt->delta() > 0)
			m_range *= 1.1;
		else
			m_range /= 1.1;

		onSizeChanged();
		updateBars(m_lastMat);
	} else
		QGraphicsPixmapItem::wheelEvent(evt);
}

void ActivityVisualizerBars::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
	if (evt->button() & Qt::MiddleButton) {
		m_range = 1.0;
		onSizeChanged();
		updateBars(m_lastMat);
	} else
		ActivityVisualizer::mousePressEvent(evt);
}

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
		static QColor blue(51, 153, 255);
		static QColor red(246, 2, 2);

		if (m_barsOrientation == HORIZONTAL) {
			m_painter.fillRect(QRect(0, 0, cols, 100), QColor(255, 255, 255));

			for (int i = 0; i < cols; i += 1) {
				qreal capped = mat->at(i);
				capped = capped > m_range ? m_range : (capped < -m_range ? -m_range : capped);

				if (capped >= 0) {
					qreal span = 50.0 * capped / m_range;
					m_painter.fillRect(QRectF(i, 50 - span, 1, span), blue.rgb());
				} else {
					qreal span = -50.0 * capped / m_range;
					m_painter.fillRect(QRectF(i, 50, 1, span), red.rgb());
				}
			}
		} else {
			m_painter.fillRect(QRect(0, 0, 100, rows), QColor(255, 255, 255));

			for (int j = 0; j < rows; j += 1) {
				qreal capped = mat->at(j);
				capped = capped > m_range ? m_range : (capped < -m_range ? -m_range : capped);

				if (capped >= 0) {
					qreal span = 50.0 * capped / m_range;
					m_painter.fillRect(QRectF(50, j, span, 1), blue.rgb());
				} else {
					qreal span = -50.0 * capped / m_range;
					m_painter.fillRect(QRectF(50 - span, j, span, 1), red.rgb());
				}
			}
		}

		// Update pixmap from image
		updatePixmap();
	} else {
		qWarning() << "Invalid number of data to update bars: "
		           << mat->size() << "data points for" << rows << "x" << cols;
	}

	// Display the scalar value when it's a single scalar
	if (m_box->outputType() == SCALAR || (m_box->outputType() == MATRIX && m_box->rows() == 1 && m_box->cols() == 1)) {
		m_scalarValue.setHtml(QString("<center>%1</center>").arg(QString::number(mat->at(0))));
		m_scalarValue.setPos(0, m_height / 2);
	}

	// Delete the last store matrix and replace with this one, if they are not the same
	if (m_lastMat != mat) {
		delete m_lastMat;
		m_lastMat = mat;
	}
}

void ActivityVisualizerBars::onBoxDeleted()
{
	delete this;
}

/**
 * @brief ActivityVisualizerBars::updateLines is called when the graph window is resized.
 */
void ActivityVisualizerBars::onSizeChanged()
{
	if (m_barsOrientation == HORIZONTAL) {
		// Create a horizontal line (axis)
		m_hLine.setLine((qreal)-m_scaleMargin, m_height / 2.0, (qreal) m_width + m_scaleMargin, m_height / 2.0);

		// Create a vertical line (axis)
		m_vLine.setLine((qreal)-m_scaleMargin, (qreal)-m_scaleMargin, (qreal)-m_scaleMargin, (qreal)m_height + m_scaleMargin);

		qreal dist = m_height / (m_nbTicks - 1);
		qreal tickDiff = 2 * m_range / (m_nbTicks - 1);
		for (int i = 0; i < m_nbTicks; i += 1) {
			m_ticks.at(i)->setLine(-m_scaleMargin,
			                       i * dist,
			                       -m_scaleMargin / 2,
			                       i * dist);

			m_labels.at(i)->setPlainText(QString::number(m_range - i * tickDiff, 'g', 3));
			QRectF r = m_labels.at(i)->boundingRect();
			m_labels.at(i)->setPos(-m_scaleMargin - r.width(), // right align
			                       i * dist - r.height() / 2); // middle align
		}

		// Place ticks denoting first, last and middle neurons
		const static qreal tickSpan = 5.0;

		// Place beginning tick
		// The small -0.5 offset is so that first neuron is perfectly tangeant to the extremity of the tick
		m_beginTick.setLine(-0.5, m_height / 2.0 - tickSpan / 2.0,
		                    -0.5, m_height / 2.0 + tickSpan / 2.0);

		// Place middle tick
		m_middleTick.setLine(m_width / 2.0, m_height / 2.0 - tickSpan / 2.0,
		                     m_width / 2.0, m_height / 2.0 + tickSpan / 2.0);

		// Place end tick
		m_endTick.setLine(m_width + 0.5, m_height / 2.0 - tickSpan / 2.0,
		                  m_width + 0.5, m_height / 2.0 + tickSpan / 2.0);

	} else {
		// Create a vertical line (axis)
		m_hLine.setLine(m_width / 2.0, (qreal) -m_scaleMargin, m_width / 2.0, (qreal) m_height + m_scaleMargin);

		// Create a horitzontal line (axis)
		m_vLine.setLine((qreal)-m_scaleMargin, (qreal)-m_scaleMargin, (qreal)m_width + m_scaleMargin, (qreal)-m_scaleMargin);

		qreal dist = m_width / (m_nbTicks - 1);
		qreal tickDiff = 2 * m_range / (m_nbTicks - 1);
		for (int i = 0; i < m_nbTicks; i += 1) {
			m_ticks.at(i)->setLine(i * dist,
			                       -m_scaleMargin,
			                       i * dist,
			                       -m_scaleMargin / 2);

			m_labels.at(i)->setPlainText(QString::number(m_range - i * tickDiff, 'g', 3));
			QRectF r = m_labels.at(i)->boundingRect();
			m_labels.at(i)->setRotation(90);
			m_labels.at(i)->setPos((m_nbTicks - 1 - i) * dist + r.height() / 2,      // middle align
			                       -m_scaleMargin - r.width()); // bottom align
		}

		// Place ticks denoting first, last and middle neurons
		const static qreal tickSpan = 5.0;

		// Place beginning tick
		// The small -0.5 offset is so that first neuron is perfectly tangeant to the extremity of the tick
		m_beginTick.setLine(m_width / 2.0 - tickSpan / 2.0, -0.5,
		                    m_width / 2.0 + tickSpan / 2.0, -0.5);

		// Place middle tick
		m_middleTick.setLine(m_width / 2.0 - tickSpan / 2.0, m_height / 2.0,
		                     m_width / 2.0 + tickSpan / 2.0, m_height / 2.0);

		// Place end tick
		m_endTick.setLine(m_width / 2.0 - tickSpan / 2.0, m_height + 0.5,
		                  m_width / 2.0 + tickSpan / 2.0, m_height + 0.5);
	}

	// The only way to center text is to use setHtml() AND set the TextWidth
	m_visuTitle.setTextWidth(m_width);

	m_visuTitle.setPos(0, m_height);

	// Display the scalar value when it's a single scalar value
	if (m_box->outputType() == SCALAR || (m_box->outputType() == MATRIX && m_box->rows() == 1 && m_box->cols() == 1)) {
		m_scalarValue.setTextWidth(m_width);
	}
}
