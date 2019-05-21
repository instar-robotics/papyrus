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

#include "deletelinkcommand.h"
#include "helpers.h"

DeleteLinkCommand::DeleteLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_scene(scene),
      m_link(link),
      m_outputSlot(nullptr),
      m_inputSlot(nullptr)
{
	// Sanity check
	if (m_scene == nullptr)
		informUserAndCrash(QObject::tr("Cannot delete a Link: no scene!"));

	if (m_link == nullptr)
		informUserAndCrash(QObject::tr("Cannot delete a Link: no link!"));

	if (m_link->from() == nullptr)
		informUserAndCrash(QObject::tr("Problem while deleting a Link: Link doesn't have an originating box"));

	if (m_link->to() == nullptr)
		informUserAndCrash(QObject::tr("Problem while deleting a Link: Link doesn't have a target box"));

	// Save the output slot
	m_outputSlot = m_link->from();

	// Save its input slot
	m_inputSlot = m_link->to();
}

void DeleteLinkCommand::undo()
{
	// Add back the Link to the scene
	m_scene->addItem(m_link);

	// Add back the lines to the scene
	m_link->addLinesToScene();

	// Add back the Link to its input slot
	m_inputSlot->addInput(m_link, true);

	// Add back the Link to its output slot
	m_outputSlot->addOutput(m_link);

	m_scene->update();
}

void DeleteLinkCommand::redo()
{
	// Remove this Link from its output slot
	m_outputSlot->removeOutput(m_link);

	// Remove this Link from its input slots
	m_inputSlot->removeInput(m_link);

	// Remove the lines from the scene
	m_link->removeLinesFromScene();

	// Remove the Link from the scene
	m_scene->removeItem(m_link);

	m_scene->update();
}
