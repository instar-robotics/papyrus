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

#ifndef UPDATEBOXCOMMAND_H
#define UPDATEBOXCOMMAND_H

#include "propertiespanel.h"
#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The UpdateZoneCommand class represents updating a @DiagramBox's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateBoxCommand : public QUndoCommand
{
public:
	UpdateBoxCommand(PropertiesPanel *panel, DiagramBox *box, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	DiagramBox *m_box;        // The box which we are modifying parameters

	// Old parameters (ones the box had before updating its parameters)
	QString m_oldTitle;
	int m_oldRows;
	int m_oldCols;
	bool m_oldActivity;
	bool m_oldPublish;
	QString m_oldTopic;
	VisuType m_oldVisuType;

	// New parameters (ones the box will have after updating its parameters)
	QString m_newTitle;
	int m_newRows;
	int m_newCols;
	bool m_newActivity;
	bool m_newPublish;
	QString m_newTopic;
	VisuType m_newVisuType;
};

#endif // UPDATEBOXCOMMAND_H
