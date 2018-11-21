#include "scalarvisualization.h"
#include "scalarfetcher.h"
#include "matrixfetcher.h"
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

	createCharts();

	m_vLayout = new QVBoxLayout;
//	m_vLayout->addWidget(m_menuBar);
	m_vLayout->addWidget(m_barView);
	m_vLayout->addWidget(m_graphView);
	m_graphView->hide();
	m_vLayout->setContentsMargins(0, 35, 0, 0);
	setLayout(m_vLayout);

	// Create the appropriate fetcher
	QString fullTopicName = mkTopicName(m_box->scriptName(), m_box->topic());
	if (m_box->outputType() == SCALAR)
		m_dataFetcher = new ScalarFetcher(fullTopicName, this, BAR);
	else if (m_box->outputType() == MATRIX)
		m_dataFetcher = new MatrixFetcher(fullTopicName, this, BAR);
	else
		informUserAndCrash(tr("[ScalarVisualization] Output type not supported, supported are SCALAR"
		                      " and MATRIX"));
}

void ScalarVisualization::mousePressEvent(QMouseEvent *evt)
{
	Q_UNUSED(evt);
	qDebug() << "Click detected, what to do?^^";
	qDebug() << "Layout Content Margins:" << layout()->contentsMargins();
}

void ScalarVisualization::updateBarValues(const std::vector<qreal> &values)
{
	bool rangeChanged = false;
	qreal thisMax = values.at(0);
	qreal thisMin = values.at(0);

	for (unsigned int i = 0; i < values.size(); i += 1) {
		m_barSets.at(i)->replace(0, values.at(i));

		if (values.at(i) > thisMax)
			thisMax = values.at(i);
		if (values.at(i) < thisMin)
			thisMin = values.at(i);

	}
	// Update the range (for now, range doesn't shrink back)
	if (thisMax > m_barMax) {
		m_barMax = 1.2 * thisMax;
		rangeChanged = true;
	}
	if (thisMin < m_barMin) {
		if (thisMin < 0)
			m_barMin = 1.2 * thisMin;
		else
			m_barMin = 0.8 * thisMin;
		rangeChanged = true;
	}

	if (rangeChanged)
		m_barAxisY->setRange(m_barMin, m_barMax);
}

void ScalarVisualization::pushGraphValues(const std::vector<qreal> &values)
{
	bool rangeChanged = false;
	qreal thisMax = values.at(0);
	qreal thisMin = values.at(0);

	// Compute distance to scroll the graph by
	qreal dx = m_graphChart->plotArea().width() / (500);
	m_idx += (m_graphAxisX->max() - m_graphAxisX->min()) / (500);
	m_graphChart->scroll(dx, 0);

	// Append values to the the series
	for (unsigned int i = 0; i < values.size(); i += 1) {
		m_graphSeries.at(i)->append(m_idx, values.at(i));

		if (values.at(i) > thisMax)
			thisMax = values.at(i);
		if (values.at(i) < thisMin)
			thisMin = values.at(i);
	}

	// Update the range (for now, range doesn't shrink back)
	if (thisMax > m_graphMax) {
		m_graphMax = 1.2 * thisMax;
		rangeChanged = true;
	}
	if (thisMin < m_graphMin) {
		if (thisMin < 0)
			m_graphMin = 1.2 * thisMin;
		else
			m_barMin = 0.8 * thisMin;
		rangeChanged = true;
	}

	if (rangeChanged)
		m_graphAxisY->setRange(m_graphMin, m_graphMax);
}

void ScalarVisualization::createCharts()
{
	// First, check the size
	if (m_box == nullptr) {
		qCritical() << "Cannot create charts: no box";
		return;
	}

	if (m_box->outputType() == SCALAR) {
		m_size = 1;
	} else if (m_box->outputType() == MATRIX && m_box->rows() == 1) {
		m_size = m_box->cols();
	} else if (m_box->outputType() == MATRIX && m_box->cols() == 1) {
		m_size = m_box->rows();
	} else {
		qCritical() << "Cannot create charts: not a scalar nor a vector!";
		qDebug() << "rows:" << m_box->rows() << "cols:" << m_box->cols();
		return;
	}

	qDebug() << "Created charts with size" << m_size;

	// Create the chart for BAR mode
	for (int i = 0; i < m_size; i += 1) {
		QBarSet *s = new QBarSet(QString::number(i));
		QColor barColor(51, 153, 255);
		s->setColor(barColor);
		*s << 0;
		m_barSets.append(s);
	}

	m_barSeries = new QBarSeries;
	m_barSeries->append(m_barSets);

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
	m_graphChart = new QChart;

	for (int i = 0; i < m_size; i += 1) {
		m_graphSeries.append(new QSplineSeries);
		m_graphChart->addSeries(m_graphSeries.at(i));
	}

	m_graphChart->setAnimationOptions(QChart::NoAnimation);
	m_graphChart->createDefaultAxes();
	m_graphAxisX = new QValueAxis;
	m_graphAxisY = new QValueAxis;
	m_graphAxisX->setRange(-500, 0);
	m_graphAxisX->setTickCount(6);

	m_graphAxisY->setRange(m_graphMin, m_graphMax);
	m_graphAxisY->setTickCount(5);

	for (int i = 0; i < m_size; i += 1) {
		m_graphChart->setAxisX(m_graphAxisX, m_graphSeries.at(i));
		m_graphChart->setAxisY(m_graphAxisY, m_graphSeries.at(i));
	}

	m_graphChart->legend()->hide();
	m_graphChart->setMargins(QMargins(2, 2, 2, 2));
	m_graphView = new QChartView(m_graphChart);
	m_graphView->setRenderHint(QPainter::Antialiasing);
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
