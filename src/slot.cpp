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

#include "slot.h"

Slot::Slot(QGraphicsItem *parent) : QGraphicsItem(parent), m_dist(0), m_box(NULL)
{
	m_name.clear();
}

Slot::Slot(QString &name, QGraphicsItem *parent) : Slot(parent)
{
	setName(name);
}

Slot::~Slot()
{
}

QString Slot::name() const
{
	return m_name;
}

void Slot::setName(const QString &name)
{
	m_name = name;
}

qreal Slot::dist() const
{
	return m_dist;
}

void Slot::setDist(const qreal &dist)
{
	m_dist = dist;
}

DiagramBox *Slot::box() const
{
	return m_box;
}

void Slot::setBox(DiagramBox *box)
{
	m_box = box;
}
