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

#include "addboxcommand.h"

#include <QDebug>

AddBoxCommand::AddBoxCommand(DiagramScene *scene, DiagramBox *box, const QPointF &initialPos,
                             QUndoCommand *parent)
    : QUndoCommand(parent),
      m_scene(scene),
      m_box(box),
      m_initialPos(initialPos)
{
	if (box == nullptr) {
		qWarning() << "[AddBoxCommand] created with a null pointer in place of box!";
		return;
	}

	if (scene == nullptr) {
		qWarning() << "[AddBoxCommand] created with a null pointer in place of scene!";
		return;
	}
}

AddBoxCommand::~AddBoxCommand()
{
	// If we have a box in the pointer and this box is not in the scene, we need to destroy it,
	// because we are the only one with this ressource
	if (m_box != nullptr && m_box->scene() == nullptr) {
		delete m_box;
		m_box = nullptr;
	}
}

void AddBoxCommand::undo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddBoxCommand] cannot undo: null pointer for scene!";
		return;
	}

	// do not use deleteItem() because we only want to remove it from the scene, not destroy it
	m_scene->removeItem(m_box);
	// No need to remove parent: it loses its parentItem when it's removed from the scene
	m_scene->update();
}

void AddBoxCommand::redo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddBoxCommand] cannot redo: null pointer for scene!";
		return;
	}

	m_scene->addBox(m_box, m_initialPos);

	// Check if the box was dropped on a zone
	foreach (QGraphicsItem *item, m_box->collidingItems()) {
		Zone *zone = dynamic_cast<Zone *>(item);

		if (zone != nullptr) {
			// Add the box to the zone
			QPointF sP = m_box->scenePos();
			m_box->setParentItem(zone);
			m_box->setPos(zone->sceneTransform().inverted().map(sP));
			break;
		}
	}
}
