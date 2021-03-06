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

#include "addzonecommand.h"

#include <QDebug>

AddZoneCommand::AddZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_scene(scene),
      m_zone(zone)
{

}

AddZoneCommand::~AddZoneCommand()
{
	// If the zone is not in the scene, we need to destroy it now, because at this point we are the
	// last reference to it
	if (m_zone != nullptr && m_zone->scene() == nullptr) {
		delete m_zone;
		m_zone = nullptr;
	}
}

void AddZoneCommand::undo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddZoneCommand] cannot undo: null pointer for scene!";
		return;
	}

	if (m_zone == nullptr) {
		qWarning() << "[AddZoneCommand] cannot undo: null pointer for zone!";
		return;
	}

	// First, remove the zone as a parent from all its children
	foreach (QGraphicsItem *child, m_zone->childItems()) {
		child->setParentItem(nullptr);
		child->setSelected(false);
	}

	m_scene->removeItem(m_zone);

	if (m_scene->script() != nullptr)
		m_scene->script()->setStatusModified(true);
}

void AddZoneCommand::redo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddZoneCommand] cannot redo: null pointer for scene!";
		return;
	}

	if (m_zone == nullptr) {
		qWarning() << "[AddZoneCommand] cannot redo: null pointer for zone!";
		return;
	}

	m_scene->addItem(m_zone);
	m_zone->updateGroup();

	m_zone->moveBy(0.1, 0); // Dirty trick to trigger the itemChange() and snap position on the grid
	foreach (QGraphicsItem *item, m_zone->childItems()) {
		item->setSelected(false);
	}

	if (m_scene->script() != nullptr)
		m_scene->script()->setStatusModified(true);
}
