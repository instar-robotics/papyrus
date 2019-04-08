/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER
 
  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.
 
  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

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
