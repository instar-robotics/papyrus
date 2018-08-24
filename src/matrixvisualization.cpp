#include "matrixvisualization.h"
#include "matrixfetcher.h"
#include "diagrambox.h"
#include "helpers.h"

#include <QDebug>

MatrixVisualization::MatrixVisualization(QWidget *parent, QGraphicsScene *scene, DiagramBox *box) :
    DataVisualization(parent, scene, box),
    m_grayImageLabel(nullptr)
{
	qDebug() << "[MatrixVisualization] created";

	// TEMPORARY
	if (!m_grayImage.load("/home/nschoe/pictures/media.jpg"))
		qCritical() << "Failed to load image";

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

	QString fullTopicName = mkTopicName(m_box->scriptName(), m_box->topic());
	m_dataFetcher = new MatrixFetcher(fullTopicName, this, GRAYSCALE);

}

void MatrixVisualization::updateGrayscale(const std::vector<qreal> &values)
{
	qDebug() << "[UpdateGrayscale] with" << values.size() << "values";
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
