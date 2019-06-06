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

#ifndef ADDZONECOMMAND_H
#define ADDZONECOMMAND_H

#include "diagramscene.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The AddZoneCommand class represents adding a @Zone to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddZoneCommand : public QUndoCommand
{
public:
	AddZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent = nullptr);
	~AddZoneCommand();

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene;  // The scene in which to add the zone
	Zone *m_zone;           // The zone to add to the scene
};

#endif // ADDZONECOMMAND_H
