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

#include "helpers.h"
#include "updatezonecommand.h"

#include <QDebug>

UpdateZoneCommand::UpdateZoneCommand(PropertiesPanel *panel, Zone *zone, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_panel(panel),
      m_zone(zone)
{
	if (m_zone == nullptr) {
		qWarning() << "[UpdateZoneCommand] created with null pointer in place of zone!";
		return;
	}

	if (m_panel == nullptr) {
		qWarning() << "[UpdateZoneCommand] created with null pointer in place of panel!";
	}

	// Save the current parameter values
	m_oldTitle = m_zone->title();
	m_oldColor = m_zone->color();

	// Save new parameter values
	m_newTitle = m_panel->getZoneTitle();
	m_newColor = m_panel->getZoneColor();
}

void UpdateZoneCommand::undo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot undo: null pointer for panel!";
		return;
	}

	if (m_zone == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot undo: null pointer for zone!";
		return;
	}

	m_zone->setTitle(m_oldTitle);
	m_zone->setColor(m_oldColor);

	m_zone->update();

	if (m_zone->scene() != nullptr) {
		DiagramScene *dScene = dynamic_cast<DiagramScene *>(m_zone->scene());

		if (dScene != nullptr && dScene->script() != nullptr)
			dScene->script()->setStatusModified(true);
	}
}

void UpdateZoneCommand::redo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot redo: null pointer for panel!";
		return;
	}

	if (m_zone == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot redo: null pointer for zone!";
		return;
	}

	m_zone->setTitle(m_newTitle);
	m_zone->setColor(m_newColor);

	m_zone->update();

	if (m_zone->scene() != nullptr) {
		DiagramScene *dScene = dynamic_cast<DiagramScene *>(m_zone->scene());

		if (dScene != nullptr && dScene->script() != nullptr)
			dScene->script()->setStatusModified(true);
	}
}
