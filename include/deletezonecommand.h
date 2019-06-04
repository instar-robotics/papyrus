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

#ifndef DELETEZONECOMMAND_H
#define DELETEZONECOMMAND_H

#include "zone.h"
#include "diagramscene.h"
#include "zone.h"

#include <QUndoCommand>

class DeleteZoneCommand : public QUndoCommand
{
public:
	DeleteZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent = nullptr);
	~DeleteZoneCommand();

	void undo() override;
	void redo() override;

private:
	DiagramScene *m_scene;
	Zone *m_zone;
	QList<QGraphicsItem *> m_children;
};

#endif // DELETEZONECOMMAND_H
