#include "scalarvisualization.h"
#include "scalarfetcher.h"
#include "matrixfetcher.h"
#include "helpers.h"
#include "types.h"

#include <QDebug>
#include <QPainter>
#include <QColor>

ScalarVisualization::ScalarVisualization(QWidget *parent,
                                         ROSSession *rosSession,
                                         QGraphicsScene *scene,
                                         DiagramBox *box) :
    DataVisualization(parent, rosSession, scene, box),
    m_idx(0),
    m_barMin(-1),
    m_barMax(1),
    m_graphMin(-1),
    m_graphMax(1)
{
	// We populate the available types of visualization for Scalar in the menu
	m_typeMenu->addAction(tr("Bar"), this, SLOT(switchToBar()));
	m_typeMenu->addAction(tr("Graph"), this, SLOT(switchToGraph()));

	createCharts();

	m_vLayout = new QVBoxLayout;
//	m_vLayout->addWidget(m_menuBar);
	m_vLayout->addWidget(m_barView);
	m_vLayout->addWidget(m_graphView);
	m_graphView->hide();
//	m_vLayout->setContentsMargins(0, 35, 0, 0);
	m_vLayout->setContentsMargins(0, 40, 0, 0);
	setLayout(m_vLayout);

	// Create the appropriate fetcher
	if (m_box->outputType() == SCALAR) {
		// Give the correct topic name (either one which is set on the box's parameters, or the
		// computed topic name, if the topic is not published)
		if (m_box->publish())
			m_dataFetcher = new ScalarFetcher(m_box->topic(), this, BAR);
		else {
			m_dataFetcher = new ScalarFetcher(ensureSlashPrefix(mkTopicName(m_box->scriptName(), m_box->uuid().toString())), this, BAR);
			// Activate the topic
			m_rosSession->addToHotList(m_box->uuid());
		}
	}
	else if (m_box->outputType() == MATRIX)
		if (m_box->publish())
			m_dataFetcher = new MatrixFetcher(m_box->topic(), this, BAR);
		else {
			m_dataFetcher = new MatrixFetcher(ensureSlashPrefix(mkTopicName(m_box->scriptName(), m_box->uuid().toString())), this, BAR);
			// Activate the topic
			m_rosSession->addToHotList(m_box->uuid());
		}
	else
		informUserAndCrash(tr("[ScalarVisualization] Output type not supported, supported are SCALAR"
		                      " and MATRIX"));
}

void ScalarVisualization::mousePressEvent(QMouseEvent *evt)
{
	Q_UNUSED(evt);
//	qDebug() << "Click detected, what to do?^^";
//	qDebug() << "Layout Content Margins:" << layout()->contentsMargins();
}

void ScalarVisualization::updateBarValues(const std::vector<qreal> &values)
{
	bool rangeChanged = false;
	qreal thisMax = values.at(0);
	qreal thisMin = values.at(0);

	for (unsigned int i = 0; i < values.size(); i += 1) {
		m_barSet->replace(i, values.at(i));

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

	if (rangeChanged) {
		qreal bound = qrealAbsMax(m_barMin, m_barMax);

		if (m_matrixShape == POINT || m_matrixShape == ROW_VECT)
			m_barAxisY->setRange(-bound, bound);
		else if (m_matrixShape == COL_VECT)
			m_barAxisX->setRange(-bound, bound);
		else
			informUserAndCrash(tr("Unsupported matrix shape when pushing bar balues"));
	}
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

	if (rangeChanged) {
		qreal bound = qrealAbsMax(m_graphMin, m_graphMax);
		m_graphAxisY->setRange(-bound, bound);
	}
}

void ScalarVisualization::createCharts()
{
	// First, check the size
	if (m_box == nullptr) {
		informUserAndCrash(tr("Cannot create charts: no box"));
		return;
	}

	if (m_box->outputType() == SCALAR) {
		m_size = 1;
		m_matrixShape = POINT;
	} else if (m_box->outputType() == MATRIX && m_box->rows() == 1) {
		m_size = m_box->cols();
		m_matrixShape = ROW_VECT;
	} else if (m_box->outputType() == MATRIX && m_box->cols() == 1) {
		m_size = m_box->rows();
		m_matrixShape = COL_VECT;
	} else {
		informUserAndCrash(tr("Cannot create charts: not a scalar nor a vector!"));
		return;
	}

	// Create the chart for BAR mode
	m_barSet = new QBarSet("Neurons");
	m_barSet->setColor(QColor(51, 153, 255));
	for (int i = 0; i < m_size; i += 1) {
		*m_barSet << 0; // initialize values at zero
	}

	m_barChart = new QChart;

	// VERTICAL (row vector)
	if (m_matrixShape == POINT || m_matrixShape == ROW_VECT) {
		m_barSeries = new QBarSeries;
		m_barSeries->append(m_barSet);
		m_barSeries->setLabelsVisible(false);


		m_barChart->addSeries(m_barSeries);

		m_barAxisY = new QValueAxis;
		m_barAxisY->setRange(m_barMin, m_barMax);
		m_barAxisY->setTickCount(9);

		m_barChart->setAxisY(m_barAxisY, m_barSeries);
	} else if (m_matrixShape == COL_VECT) {
		m_horizontalBarSeries = new QHorizontalBarSeries;
		m_horizontalBarSeries->append(m_barSet);
		m_horizontalBarSeries->setLabelsVisible(false);

		m_barChart->addSeries(m_horizontalBarSeries);

		m_barAxisX = new QValueAxis;
		m_barAxisX->setRange(m_barMin, m_barMax);
		m_barAxisX->setTickCount(9);

		m_barChart->setAxisX(m_barAxisX, m_horizontalBarSeries);
	}

	m_barChart->setAnimationOptions(QChart::NoAnimation);

	m_barChart->legend()->hide();
	m_barChart->setMargins(QMargins(0, 0, 0, 0));

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
