#include "matrixvisualization.h"
#include "matrixfetcher.h"
#include "diagrambox.h"
#include "helpers.h"

#include <QDebug>
#include <QVector>
#include <QPainter>

MatrixVisualization::MatrixVisualization(QWidget *parent, QGraphicsScene *scene, DiagramBox *box) :
    DataVisualization(parent, scene, box),
    m_grayImageLabel(nullptr),
//    m_grayImage(QImage(100, 100, QImage::Format_Indexed8))
    m_grayImage(QImage(box->cols(), box->rows(), QImage::Format_Indexed8)) // nullptr checked in parent
{
	qDebug() << "[MatrixVisualization] created";

	qDebug() << "Image size:" << m_grayImage.size();

	m_grayImage.fill(qRgb(128, 128, 128));

	// First, initialize the color table with the 256 shades of gray
	for (unsigned int i = 0; i < 256; i += 1) {
		m_grayImage.setColor(i, qRgb(i, i, i));
	}

	// We populate the available types of visualization for Scalar in the menu
	m_typeMenu->addAction(tr("Grayscale"), this, SLOT(switchToGrayscale()));
	m_typeMenu->addAction(tr("Image"), this, SLOT(switchToImage()));
	m_typeMenu->addAction(tr("Landscape"), this, SLOT(switchToLandscape()));

	m_grayImageLabel = new QLabel;
	m_grayImageLabel->setPixmap(QPixmap::fromImage(m_grayImage));
//	m_grayImageLabel->setBackgroundRole(QPalette::Base);
	m_grayImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	m_grayImageLabel->setScaledContents(true);
	m_grayImageLabel->adjustSize();

	m_vLayout = new QVBoxLayout;
	m_vLayout->addWidget(m_grayImageLabel);
	m_vLayout->setContentsMargins(0, 35, 0, 0);
	setLayout(m_vLayout);

	m_dataFetcher = new MatrixFetcher(m_box->topic(), this, GRAYSCALE);
	connect(m_dataFetcher, SIGNAL(newMatrix(QList<double>*)), this, SLOT(updateGrayscale(QList<double>*)));
}

void MatrixVisualization::updateGrayscale(QList<double> *values)
{
	int cols = m_box->cols();
	int rows = m_box->rows();

	if (values->size() == cols * rows) {
		for (int i = 0; i < cols; i += 1) {
			for (int j = 0; j < rows; j += 1) {
				int idx = values->at(i + j) * 255;
				if (idx > 255)
					idx = 255;
				if (idx < 0)
					idx = 0;
				m_grayImage.setPixel(i, j, idx);
			}
		}
		m_grayImageLabel->setPixmap(QPixmap::fromImage(m_grayImage));
	} else {
		qCritical() << "Invalid number of data to update grayscale image";
	}
}

void MatrixVisualization::switchToGrayscale()
{

}

void MatrixVisualization::switchToImage()
{

}

void MatrixVisualization::switchToLandscape()
{

}
