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

DataVisualization::DataVisualization(QWidget *parent,
                                     ROSSession *rosSession,
                                     QGraphicsScene *scene,
                                     DiagramBox *box) :
    QWidget(parent),
    m_rosSession(rosSession),
    m_scene(scene),
    m_box(box),
    m_dataFetcher(nullptr)
{
	if (m_box == nullptr)
		informUserAndCrash(tr("Data visualization has no box!"),
		                   tr("A DataVisualization was created without a DiagramBox. This should not"
		                      " happen."));

	if (m_rosSession == nullptr)
		informUserAndCrash(tr("No associated ROS Session"),
		                   tr("A DataVisualization was created without a ROSSession. This should not"
		                      " happen"));

//	m_menuBar = new QMenuBar;
	m_menuBar = new QMenuBar(this);
//	m_menuBar->setMinimumHeight(30);
	m_menuBar->setFixedHeight(40);
	m_typeMenu = m_menuBar->addMenu(tr("Type"));
}

DataVisualization::~DataVisualization()
{
	if (m_dataFetcher != nullptr) {
		m_dataFetcher->setShouldQuit(true);
		m_dataFetcher->wait(1000);
	}
}
