#include "scalarvisualization.h"
#include "scalarfetcher.h"
#include "helpers.h"
#include "types.h"

#include <QDebug>
#include <QPainter>
#include <QColor>

ScalarVisualization::ScalarVisualization(QWidget *parent, QGraphicsScene *scene, DiagramBox *box) :
    DataVisualization(parent, scene, box),
    m_idx(0),
    m_barMin(-1),
    m_barMax(1),
    m_graphMin(-1),
    m_graphMax(1)
{
	qDebug() << "[ScalarVis] created";
	// We populate the available types of visualization for Scalar in the menu
	m_typeMenu->addAction(tr("Bar"), this, SLOT(switchToBar()));
	m_typeMenu->addAction(tr("Graph"), this, SLOT(switchToGraph()));

	// Create the chart for BAR mode
	m_barSet = new QBarSet(tr("Neuron"));
	*m_barSet << 0;
	m_barSeries = new QBarSeries;
	m_barSeries->append(m_barSet);
	m_barChart = new QChart;
	m_barChart->addSeries(m_barSeries);
	m_barChart->setAnimationOptions(QChart::SeriesAnimations);
	m_barChart->createDefaultAxes();
	m_barAxisY = new QValueAxis;
	m_barAxisY->setRange(m_barMin, m_barMax);
	m_barAxisY->setTickCount(5);
	m_barChart->setAxisY(m_barAxisY, m_barSeries);
	m_barChart->legend()->hide();
	m_barChart->setMargins(QMargins(2, 2, 2, 2));
	m_barView = new QChartView(m_barChart);
	m_barView->setRenderHint(QPainter::Antialiasing);

	// Create the chart for GRAPH mode
	m_graphSeries = new QSplineSeries;
	m_graphChart = new QChart;
	m_graphChart->addSeries(m_graphSeries);
	m_graphChart->setAnimationOptions(QChart::NoAnimation);
	m_graphChart->createDefaultAxes();
	m_graphAxisX = new QValueAxis;
	m_graphAxisY = new QValueAxis;
	m_graphAxisX->setRange(-500, 0);
	m_graphAxisX->setTickCount(6);
	m_graphChart->setAxisX(m_graphAxisX, m_graphSeries);
	m_graphAxisY->setRange(m_graphMin, m_graphMax);
	m_graphAxisY->setTickCount(5);
	m_graphChart->setAxisY(m_graphAxisY, m_graphSeries);
	m_graphChart->legend()->hide();
	m_graphChart->setMargins(QMargins(2, 2, 2, 2));
	m_graphView = new QChartView(m_graphChart);
	m_graphView->setRenderHint(QPainter::Antialiasing);

	m_vLayout = new QVBoxLayout;
	m_vLayout->addWidget(m_menuBar);
	m_vLayout->addWidget(m_barView);
	m_vLayout->addWidget(m_graphView);
	m_graphView->hide();
	m_vLayout->setContentsMargins(0, 0, 0, 0);
	setLayout(m_vLayout);

	QString fullTopicName = mkTopicName(m_box->scriptName(), m_box->topic());
	m_dataFetcher = new ScalarFetcher(fullTopicName, this, BAR);
}

void ScalarVisualization::mousePressEvent(QMouseEvent *evt)
{
	qDebug() << "Click detected, what to do?^^";
	qDebug() << "Layout Content Margins:" << layout()->contentsMargins();
}

void ScalarVisualization::updateBarValue(const qreal value)
{
	m_barSet->replace(0, value);

	// Update the color bar based on the value
	if (value < 0)
		m_barSet->setColor(QColor(200, 0, 0, 200));
	else
		m_barSet->setColor(QColor(32, 159, 223, 200));

	bool rangeChanged = false;

	// Update the range (for now, range doesn't shrink back)
	if (value > m_barMax) {
		m_barMax = 1.2 * value;
		rangeChanged = true;
	}
	if (value < m_barMin) {
		if (value < 0)
			m_barMin = 1.2 * value;
		else
			m_barMin = 0.8 * value;
		rangeChanged = true;
	}

	if (rangeChanged)
		m_barAxisY->setRange(m_barMin, m_barMax);
}

void ScalarVisualization::pushGraphValue(const qreal value)
{
//	qreal dx = m_graphChart->plotArea().width() / (m_graphAxisX->tickCount() - 1);
//	m_idx += (m_graphAxisX->max() - m_graphAxisX->min()) / ((m_graphAxisX->tickCount() - 1));
	qreal dx = m_graphChart->plotArea().width() / (500);
	m_idx += (m_graphAxisX->max() - m_graphAxisX->min()) / (500);

	m_graphChart->scroll(dx, 0);
	m_graphSeries->append(m_idx, value);

	bool rangeChanged = false;

	// Update the range (for now, range doesn't shrink back)
	if (value > m_graphMax) {
		m_graphMax = 1.2 * value;
		rangeChanged = true;
	}
	if (value < m_graphMin) {
		if (value < 0)
			m_graphMin = 1.2 * value;
		else
			m_graphMin = 0.8 * value;
		rangeChanged = true;
	}

	if (rangeChanged)
		m_graphAxisY->setRange(m_graphMin, m_graphMax);
}

void ScalarVisualization::switchToBar()
{
	if (m_dataFetcher->visType() != BAR) {
		m_dataFetcher->setVisType(BAR);
		qDebug() << "[ScalarVis] switch to Bar";
		m_graphView->hide();
		m_barView->show();
	}
}

void ScalarVisualization::switchToGraph()
{
	if (m_dataFetcher->visType() != GRAPH) {
		m_dataFetcher->setVisType(GRAPH);
		qDebug() << "[ScalarVis] switch to Graph";
		m_barView->hide();
		m_graphView->show();
	}
}
