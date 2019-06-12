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

#ifndef UPDATEZONECOMMAND_H
#define UPDATEZONECOMMAND_H

#include "propertiespanel.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @Zone's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateZoneCommand : public QUndoCommand
{
public:
	UpdateZoneCommand(PropertiesPanel *panel, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Zone *m_zone;             // The zone which we are modifying parameters

	// Old parameters
	QString m_oldTitle;
	QColor m_oldColor;

	// New parameters
	QString m_newTitle;
	QColor m_newColor;
};

#endif // UPDATEZONECOMMAND_H
