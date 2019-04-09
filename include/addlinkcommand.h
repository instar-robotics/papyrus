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

#ifndef ADDLINKCOMMAND_H
#define ADDLINKCOMMAND_H

#include "diagramscene.h"
#include "link.h"
#include "inputslot.h"
#include "outputslot.h"

#include <QUndoCommand>

/**
 * @brief The AddLinkCommand class represents adding a @Link to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddLinkCommand : public QUndoCommand
{
public:
	AddLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the link is added
	Link *m_link;          // The link to add to the scene
	OutputSlot *m_from;    // The output slot the link is from
	InputSlot *m_to;       // The input slot the link goes to
	bool m_isFirst;        // Flag that indicates wheather a redo() action is the first one
};

#endif // ADDLINKCOMMAND_H
