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

#include "deletezonecommand.h"

DeleteZoneCommand::DeleteZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_scene(scene),
      m_zone(zone)
{

}

void DeleteZoneCommand::undo()
{
	QUndoCommand::undo();

	// Add back the zone to the scene
	m_scene->addItem(m_zone);

	// Add back the children to the zone
	foreach (QGraphicsItem *child, m_children) {
		QPointF sP = child->scenePos();
		child->setParentItem(m_zone);
		child->setPos(m_zone->sceneTransform().inverted().map(sP));
	}

	// Empty the list
	m_children.clear();
}

void DeleteZoneCommand::redo()
{
	QUndoCommand::redo();

	// Remove itself as a parent from all children
	foreach (QGraphicsItem *child, m_zone->childItems()) {
		QPointF sP = child->scenePos();
		child->setParentItem(nullptr);
		child->setPos(sP);
		m_children << child;
	}

	// Remove the item from the scene
	m_scene->removeItem(m_zone);
}
