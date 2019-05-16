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

#ifndef MOVECOMMAN_H
#define MOVECOMMAN_H

#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The MoveCommand class represents moving a @DiagramBox on a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */

class MoveCommand : public QUndoCommand
{
public:
	MoveCommand(DiagramBox *box, const QPointF &oldPos, QUndoCommand *parent = nullptr);
	void undo() override;
	void redo() override;
	void handleZone();

private:
	DiagramBox *m_box;
	QPointF m_oldPos;
	QPointF m_newPos;
};

#endif // MOVECOMMAN_H
