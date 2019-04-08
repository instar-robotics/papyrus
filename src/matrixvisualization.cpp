/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER
 
  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.
 
  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

#include "matrixvisualization.h"
#include "matrixfetcher.h"
#include "diagrambox.h"
#include "helpers.h"

#include <QDebug>
#include <QVector>
#include <QPainter>

MatrixVisualization::MatrixVisualization(QWidget *parent,
                                         ROSSession *rosSession,
                                         QGraphicsScene *scene,
                                         DiagramBox *box) :
    DataVisualization(parent, rosSession, scene, box),
    m_thermalImageLabel(nullptr),
    m_thermalImage(QImage(box->cols(), box->rows(), QImage::Format_RGB32)) // nullptr checked in parent
{
	qDebug() << "[MatrixVisualization] created";

	qDebug() << "Image size:" << m_thermalImage.size();

	// Fill image with colors corresponding to 0
	m_thermalImage.fill(qRgb(128, 128, 128));

	// We populate the available types of visualization for Scalar in the menu
	m_typeMenu->addAction(tr("Thermal"), this, SLOT(switchToThermal()));
	m_typeMenu->addAction(tr("Image"), this, SLOT(switchToImage()));
	m_typeMenu->addAction(tr("Landscape"), this, SLOT(switchToLandscape()));

	m_thermalImageLabel = new QLabel;
	m_thermalImageLabel->setPixmap(QPixmap::fromImage(m_thermalImage));
//	m_grayImageLabel->setBackgroundRole(QPalette::Base);
	m_thermalImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	m_thermalImageLabel->setScaledContents(true);
	m_thermalImageLabel->adjustSize();

	m_vLayout = new QVBoxLayout;
	m_vLayout->addWidget(m_thermalImageLabel);
	m_vLayout->setContentsMargins(0, 35, 0, 0);
	setLayout(m_vLayout);

	if (m_box->publish())
		m_dataFetcher = new MatrixFetcher(m_box->topic(), this, GRAYSCALE);
	else {
		m_dataFetcher = new MatrixFetcher(ensureSlashPrefix(mkTopicName(m_box->scriptName(), m_box->uuid().toString())), this, GRAYSCALE);
		m_rosSession->addToHotList(m_box->uuid());
	}

	connect(m_dataFetcher, SIGNAL(newMatrix(QList<double>*)), this, SLOT(updateThermal(QList<double>*)));
}

// For {0 <= x <= 0.5}
double eq1(double x) {
	return 1 / (20 * (x - 0.586)) + 0.58;
}

// For {0.5 < x <= 1}
double eq2(double x) {
	return -1 / (20 * (x - 0.414)) + 0.58;
}

void MatrixVisualization::updateThermal(QList<double> *values)
{
	if (values == nullptr) {
		qWarning() << "[updateThermal] got no data";
		return;
	}

	int cols = m_box->cols();
	int rows = m_box->rows();

	if (values->size() == cols * rows) {
		for (int i = 0; i < rows; i += 1) {
			for (int j = 0; j < cols; j += 1) {
				// Make sure the value is comprised between [-1; +1]
				double capped = values->at(j * rows + i);
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
				m_thermalImage.setPixel(j, i, value.rgb());
			}
		}
		m_thermalImageLabel->setPixmap(QPixmap::fromImage(m_thermalImage));
	} else {
		qWarning() << "Invalid number of data to update grayscale image: "
		           << values->size() << "data points for" << rows << "x" << cols;
	}
}

void MatrixVisualization::switchToThermal()
{

}

void MatrixVisualization::switchToImage()
{

}

void MatrixVisualization::switchToLandscape()
{

}
