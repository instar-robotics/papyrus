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

#ifndef SWAPBOXESCOMMAND_H
#define SWAPBOXESCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The SwapBoxesCommand class represents swapping two functions. This is used to provide
 * Undo/Redo functionality.
 */

class SwapBoxesCommand : public QUndoCommand
{
public:
	SwapBoxesCommand(DiagramScene *scene, DiagramBox *toSwap, DiagramBox *newBox,
	                 QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	DiagramScene *m_scene;  // The scene in which the swap takes place
	DiagramBox *m_toSwap;   // The box to be swapped
	DiagramBox *m_newBox;   // The box to add

	QList<Link *> m_outputLinks; // List of all links that are FROM m_toSwap
	QHash<QString, QList<Link *>> m_inputLinks; // List of all links that are TO m_toSwap, sorted by input slots
};

#endif // SWAPBOXESCOMMAND_H
