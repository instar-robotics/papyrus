#ifndef MATRIXVISUALIZATION_H
#define MATRIXVISUALIZATION_H

#include "datavisualization.h"

#include <QImage>
#include <QLabel>
#include <QVBoxLayout>


class MatrixVisualization : public DataVisualization
{
	Q_OBJECT
public:
	explicit MatrixVisualization(QWidget *parent = nullptr, QGraphicsScene *scene = nullptr, DiagramBox *box = nullptr);

	void updateGrayscale(const std::vector<qreal> &values);

private:
	QLabel *m_grayImageLabel;
	QImage m_grayImage;
	QVBoxLayout *m_vLayout;

private slots:
	void switchToGrayscale();
	void switchToImage();
	void switchToLandscape();
};

#endif // MATRIXVISUALIZATION_H
