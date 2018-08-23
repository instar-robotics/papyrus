#ifndef SCALARVISUALIZATION_H
#define SCALARVISUALIZATION_H

#include "datavisualization.h"

#include <QBarSet>
#include <QBarSeries>
#include <QChart>
#include <QChartView>
#include <QSplineSeries>
#include <QValueAxis>
#include <QVBoxLayout>

QT_CHARTS_USE_NAMESPACE

class ScalarVisualization : public DataVisualization
{
	Q_OBJECT
public:
	ScalarVisualization(QWidget *parent = nullptr, QGraphicsScene *scene = nullptr, DiagramBox *box = nullptr);
	void mousePressEvent(QMouseEvent *evt);

	void updateBarValue(const qreal value);
	void pushGraphValue(const qreal value);

private:
	QBarSet *m_barSet;
	QBarSeries *m_barSeries;
	QChart *m_barChart;
	QValueAxis *m_barAxisY;
	QChartView *m_barView;
	qreal m_barMin;
	qreal m_barMax;

	QSplineSeries *m_graphSeries;
	QChart *m_graphChart;
	QVBoxLayout *m_vLayout;
	QValueAxis *m_graphAxisX;
	QValueAxis *m_graphAxisY;
	QChartView *m_graphView;
	int m_idx;
	qreal m_graphMin;
	qreal m_graphMax;

private slots:
	void switchToBar();
	void switchToGraph();
};

#endif // SCALARVISUALIZATION_H
