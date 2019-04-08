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

#ifndef ADDBOXCOMMAND_H
#define ADDBOXCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"

#include <QUndoCommand>

/**
 * @brief The AddBoxCommand class represents adding a @DiagramBox to aa @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddBoxCommand : public QUndoCommand
{
public:
	AddBoxCommand(DiagramScene *scene, DiagramBox *box, const QPointF &initialPos, QUndoCommand *parent = nullptr);
	~AddBoxCommand();

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the box should be added
	DiagramBox *m_box;     // The DiagramBox to add to the scene
	QPointF m_initialPos;  // The initial position at which the box is added
};

#endif // ADDBOXCOMMAND_H
