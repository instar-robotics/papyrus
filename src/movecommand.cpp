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

#include "movecommand.h"
#include "zone.h"

#include <QDebug>

MoveCommand::MoveCommand(DiagramBox *box, const QPointF &oldPos, QUndoCommand *parent)
 : QUndoCommand(parent),
   m_box(box),
   m_oldPos(oldPos)
{
	if (box == nullptr) {
		qWarning() << "[MoveCommand] created with a null pointer in place of box!";
		return;
	}

	m_newPos = box->scenePos();
}

void MoveCommand::undo()
{
	if (m_box == nullptr) {
		qWarning() << "[MoveCommand] cannot undo operation: box is null!";
		return;
	}

	if (m_box->parentItem() != nullptr) {
		QPointF pPos = m_box->parentItem()->scenePos();
		m_box->setPos(m_oldPos - pPos);
	} else {
		m_box->setPos(m_oldPos);
	}

	handleZone();
}

void MoveCommand::redo()
{
	if (m_box == nullptr) {
		qWarning() << "[MoveCommand] cannot redo operation: box is null!";
		return;
	}

	if (m_box->parentItem() != nullptr) {
		QPointF pPos = m_box->parentItem()->scenePos();
		m_box->setPos(m_newPos - pPos);
	} else {
		m_box->setPos(m_newPos);
	}

	handleZone();
}

void MoveCommand::handleZone()
{
	Zone *zone = nullptr;

	// First remove itself from the current zone, to handle cases when we want to take a box outside
	// its zone
	zone = dynamic_cast<Zone *>(m_box->parentItem());
	if (zone != nullptr)
		zone->removeFromGroup(m_box);
	zone = nullptr;

	// Handle adding to/removing from zones
	foreach (QGraphicsItem *item, m_box->collidingItems()) {
		zone = dynamic_cast<Zone *>(item);
		if (zone != nullptr) {
			zone->addToGroup(m_box);
			m_box->setSelected(false);
			break;
		}
	}
}
