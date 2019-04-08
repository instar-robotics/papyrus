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

#ifndef CONSTANTDIAGRAMBOX_H
#define CONSTANTDIAGRAMBOX_H

#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QPainter>

class ConstantDiagramBox : public DiagramBox
{
	Q_OBJECT

public:
	explicit ConstantDiagramBox(const QString &name,
	                            const QIcon &icon,
	                            OutputSlot *outputSlot,
	                            const QUuid &uuid = 0,
	                            QGraphicsItem *parent = 0);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // CONSTANTDIAGRAMBOX_H
