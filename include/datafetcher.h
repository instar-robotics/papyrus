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

#ifndef DATAFETCHER_H
#define DATAFETCHER_H

#include "types.h"

#include <QThread>
#include <QString>
#include <QList>
#include <ros/ros.h>

//QT_CHARTS_USE_NAMESPACE

class DataFetcher : public QThread
{
	Q_OBJECT

public:
	explicit DataFetcher(const QString &topicName, QObject *parent = nullptr);

	bool shouldQuit() const;
	void setShouldQuit(bool shouldQuit);

	VisualizationType visType() const;
	virtual void setVisType(VisualizationType type) = 0;

protected:
	QString m_topicName;
	bool m_shouldQuit;
	VisualizationType m_visType;

signals:
	void newMatrix(QList<double> *values);
};

#endif // DATAFETCHER_H
