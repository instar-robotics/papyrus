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

#include "constants.h"
#include "addlinkcommand.h"

#include <QDebug>

AddLinkCommand::AddLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_scene(scene),
      m_link(link),
      m_from(nullptr),
      m_to(nullptr),
      m_isFirst(true)
{
	if (m_link != nullptr && m_link->from() != nullptr)
		m_from = m_link->from();

	if (m_link != nullptr && m_link->to() != nullptr)
		m_to = m_link->to();
}

void AddLinkCommand::undo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddLinkCommand] cannot undo: null pointer for scene!";
		return;
	}

	if (m_link == nullptr) {
		qWarning() << "[AddLinkCommand] cannot undo: null pointer for link!";
		return;
	}

	if (m_to != nullptr)
		m_to->removeInput(m_link);

	if (m_from != nullptr)
		m_from->removeOutput(m_link);

	m_link->removeLinesFromScene();
	m_scene->removeItem(m_link);
	m_scene->update();

	// Yes we need to set the script as modified here, even on undo, for instance when the user
	// saved the script (which sets script as unmodified) THEN hit CTRL + Z
	if (m_scene != nullptr && m_scene->script() != nullptr)
		    m_scene->script()->setStatusModified(true);
}

void AddLinkCommand::redo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddLinkCommand] cannot redo: null pointer for scene!";
		return;
	}

	if (m_link == nullptr) {
		qWarning() << "[AddLinkCommand] cannot redo: null pointer for link!";
		return;
	}

	m_scene->addItem(m_link);
	m_link->addLinesToScene();

	if (!m_isFirst) {
		if (m_from != nullptr)
			m_from->addOutput(m_link);

		if (m_to != nullptr)
			m_to->addInput(m_link);
	}

	if (m_link->checkIfInvalid() && m_scene->script() != nullptr) {
		m_scene->script()->setIsInvalid(true);
	}

	m_link->setZValue(LINKS_Z_VALUE);
	m_isFirst = false;

	if (m_scene != nullptr && m_scene->script() != nullptr)
		m_scene->script()->setStatusModified(true);
}
