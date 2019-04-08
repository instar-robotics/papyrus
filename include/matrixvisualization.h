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
