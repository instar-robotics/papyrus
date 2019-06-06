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

#ifndef DELETELINKCOMMAND_H
#define DELETELINKCOMMAND_H

#include "link.h"
#include "diagramscene.h"
#include "outputslot.h"
#include "inputslot.h"

#include <QUndoCommand>

/**
 * @brief The DeleteLinkCommand class represents deleting a @Link. This is used
 * to provide Undo/Redo functionality.
 */

class DeleteLinkCommand : public QUndoCommand
{
public:
	DeleteLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent = nullptr);
	// NOTE: we should be destroying the item in the destructor if the item is not in the scene,
	// but the undo command can be deleted when a new action is performed, at which point a further
	// undo() would segfault because the item was destroyed
//	~DeleteLinkCommand();

	void undo() override;
	void redo() override;

private:
	DiagramScene *m_scene;
	Link *m_link;
	OutputSlot *m_outputSlot;
	InputSlot *m_inputSlot;
};

#endif // DELETELINKCOMMAND_H
