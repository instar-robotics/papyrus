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

#include "deleteboxcommand.h"
#include "deletelinkcommand.h"
#include "helpers.h"

DeleteBoxCommand::DeleteBoxCommand(DiagramScene *scene, DiagramBox *box, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_scene(scene),
      m_box(box)
{
	// Sanity check
	if (m_scene == nullptr)
		informUserAndCrash(QObject::tr("Cannot delete a Box: no scene!"));

	if (m_box == nullptr)
		informUserAndCrash(QObject::tr("Cannot delete a Box: no box!"));

	m_zone = dynamic_cast<Zone *>(m_box->parentItem());
}

DeleteBoxCommand::~DeleteBoxCommand()
{
	// If the box is not in a scene, delete it now because at this point we are the last one holding
	// a pointer to it
	if (m_box != nullptr && m_box->scene() == nullptr) {
		delete m_box;
		m_box = nullptr;
	}
}

void DeleteBoxCommand::undo()
{
	QUndoCommand::undo();

	// Add back its parent (which was delete when removed from scene) if it had one
	if (m_zone != nullptr)
		m_box->setParentItem(m_zone);

	// Put the box back in the scene
	m_scene->addItem(m_box);
}

void DeleteBoxCommand::redo()
{
	QUndoCommand::redo();

	// Remove the box from the scene
	m_scene->removeItem(m_box);
}
