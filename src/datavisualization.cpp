#include "datavisualization.h"
#include "helpers.h"

#include <QDebug>
#include <QPushButton>
#include <QGraphicsProxyWidget>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QGraphicsDropShadowEffect>
#include <QDebug>
#include <QBarSet>
#include <QBarSeries>
#include <QChart>
#include <QBarCategoryAxis>
#include <QChartView>

QT_CHARTS_USE_NAMESPACE

DataVisualization::DataVisualization(QWidget *parent, QGraphicsScene *scene, DiagramBox *box) :
    QWidget(parent),
    m_scene(scene),
    m_box(box),
    m_dataFetcher(nullptr)
{
	qDebug() << "[DataVisualization] created";
	if (m_box == nullptr)
		informUserAndCrash(tr("Data visualization has no box!"),
		                   tr("A DataVisualization was created without a DiagramBox. This should not"
		                      " happen."));

//	m_menuBar = new QMenuBar(this);
	m_menuBar = new QMenuBar;
//	m_menuBar->setMinimumHeight(40);
	m_typeMenu = m_menuBar->addMenu(tr("Type"));

//	QVBoxLayout *vbox = new QVBoxLayout;

	/*
	m_dataSet = new QBarSet("Neuron");
	*m_dataSet << 1;

	QBarSeries *series = new QBarSeries;
	series->setBarWidth(0.25);
	series->append(m_dataSet);

	QChart *chart = new QChart;
	chart->addSeries(series);
	chart->setTitle("Simple static test");
	chart->setAnimationOptions(QChart::SeriesAnimations);

	QStringList categories;
	categories << "one";

	QBarCategoryAxis *axis = new QBarCategoryAxis;
	axis->append(categories);
	chart->createDefaultAxes();
	chart->setAxisX(axis, series);

	QChartView *chartView = new QChartView(chart);
	chartView->setRenderHint(QPainter::Antialiasing);

	vbox->addWidget(chartView);
	setLayout(vbox);

	QString fulltopicName = mkTopicName(m_box->scriptName(), m_box->topic());

	m_dataFetcher = new DataFetcher(fulltopicName, m_dataSet);
	*/
}

DataVisualization::~DataVisualization()
{
	qDebug() << "[~DataVisualization] destruct";
	if (m_dataFetcher != nullptr) {
		qDebug() << "[~DataVisualization] notifying & waiting for fetcher thread";
		m_dataFetcher->setShouldQuit(true);
		m_dataFetcher->wait(1000);
	}
	qDebug() << "[~DataVisualization] bye";
}
