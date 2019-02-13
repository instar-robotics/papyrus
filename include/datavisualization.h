#ifndef DATAVISUALIZATION_H
#define DATAVISUALIZATION_H

#include "datafetcher.h"
#include "rossession.h"

#include <QWidget>
#include <QMenuBar>
#include <QMenu>
#include <QGraphicsScene>
#include <QThread>
#include <QString>

//QT_CHARTS_USE_NAMESPACE

class DiagramBox;
//class QBarSet;

class DataVisualization : public QWidget
{
	Q_OBJECT

public:
	DataVisualization(QWidget *parent = nullptr,
	                  ROSSession *rosSession = nullptr,
	                  QGraphicsScene *scene = nullptr,
	                  DiagramBox *box = nullptr);
	~DataVisualization();

protected:
	ROSSession *m_rosSession;
	QGraphicsScene *m_scene;
	DiagramBox *m_box;
	QMenuBar *m_menuBar;
	QMenu *m_typeMenu;
	DataFetcher *m_dataFetcher;
};

#endif // DATAVISUALIZATION_H
