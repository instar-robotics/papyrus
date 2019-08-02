#include "activityvisualizerthermal.h"

#include <QDebug>

ActivityVisualizerThermal::ActivityVisualizerThermal(DiagramBox *box)
    : ActivityVisualizer(box)
{
	m_width = 300;
	m_height = 300;

	// Create the image
	m_image = QImage(m_box->cols(), m_box->rows(), QImage::Format_RGB32);

	// Fill with color corresponding to 0
	m_image.fill(qRgb(0, 0, 0));

	// Position the visualizer slighty above its associated box
	qreal x = m_box->scenePos().x();
	qreal y = m_box->scenePos().y() - m_height - 10;
	setPos(x, y);

//	m_painter.begin(&m_image);

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

	onSizeChanged();

	connect(this, SIGNAL(sizeChanged()), this, SLOT(onSizeChanged()));
}

void ActivityVisualizerThermal::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
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

	updateLinkToBox(evt);
	QGraphicsPixmapItem::mouseMoveEvent(evt);
}

// For {0 <= x <= 0.5}
static double eq1(double x) {
	return 1 / (20 * (x - 0.586)) + 0.58;
}

// For {0.5 < x <= 1}
static double eq2(double x) {
	return -1 / (20 * (x - 0.414)) + 0.58;
}

void ActivityVisualizerThermal::updateThermal(QVector<qreal> *mat)
{
	if (mat == nullptr) {
		qWarning() << "[updateThermal] got no data";
		return;
	}

	// Don't do anything is we're hidden (no need to take up resources)
	if (!isVisible()) {
		delete mat;
		return;
	}

	int cols = m_box->cols();
	int rows = m_box->rows();

	if (mat->size() == cols * rows) {
		for (int i = 0; i < rows; i += 1) {
			for (int j = 0; j < cols; j += 1) {
				// Make sure the value is comprised between [-1; +1]
				double capped = mat->at(j * rows + i);
				capped = capped > 1.0 ? 1.0 : (capped < -1.0 ? -1.0 : capped);

				// Normalize the value between [0; 1] for multiplication
				double normalizedValue = (capped - 1.0) / (-2.0);

				// Compute the light factor, with a two-piece equation
				double light = 0.5;

				if (0 <= normalizedValue && normalizedValue <= 0.5)
					light = eq1(normalizedValue);
				else if (0.5 < normalizedValue && normalizedValue <= 1.0)
					light = eq2(normalizedValue);
				else {
					qWarning() << "Normalized value (" << normalizedValue << ") outside of range [0, 1]";
				}

				// Make sure the equations stay within range
				light = light > 1.0 ? 1.0 : (light < 0 ? 0 : light);

				// Create the color value from the HSV scale
				// The idea is to have positive values into warm colors,
				// negative values in the cold colors
				// and black for 0.
				int hue = 180;
				if (0 <= normalizedValue && normalizedValue <= 0.5)
					hue = normalizedValue * 120; // We want from 0 (red) to 60 (yellow)
				else if (0.5 < normalizedValue && normalizedValue <= 1.0)
					hue = 359 - normalizedValue * 120;
				else {
					qWarning() << "Normalized value (" << normalizedValue << ") outside of range [0, 1]";
				}
				QColor value = QColor::fromHsl(hue, 255, light * 255);

				// Set the pixel to the value (and extract its (r,g,b) component)
				m_image.setPixel(j, i, value.rgb());
			}
		}

		// Update pixmap from image
//		setPixmap(QPixmap::fromImage(m_image).scaled(m_width, m_height));
		updatePixmap();
	} else {
		qWarning() << "Invalid number of data to update thermal image: "
		           << mat->size() << "data points for" << rows << "x" << cols;
	}

	// Don't forget to delete matrix pointer to avoid memory leak
	delete mat;
}

void ActivityVisualizerThermal::onSizeChanged()
{
	// The only way to center text is to use setHtml() AND set the TextWidth
	m_visuTitle.setTextWidth(m_width);

	m_visuTitle.setPos(0, m_height);
}
