#ifndef MATRIXVISUALIZATION_H
#define MATRIXVISUALIZATION_H

#include "datavisualization.h"
#include "rossession.h"

#include <QImage>
#include <QLabel>
#include <QVBoxLayout>


class MatrixVisualization : public DataVisualization
{
	Q_OBJECT
public:
	explicit MatrixVisualization(QWidget *parent = nullptr,
	                             ROSSession *rosSession = nullptr,
	                             QGraphicsScene *scene = nullptr,
	                             DiagramBox *box = nullptr);

private:
	QLabel *m_thermalImageLabel;
	QImage m_thermalImage;
	QVBoxLayout *m_vLayout;

private slots:
	void switchToThermal();
	void switchToImage();
	void switchToLandscape();
	void updateThermal(QList<double> *values);
};

#endif // MATRIXVISUALIZATION_H
