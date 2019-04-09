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

#ifndef UPDATESCRIPTCOMMAND_H
#define UPDATESCRIPTCOMMAND_H

#include "propertiespanel.h"
#include "script.h"

#include <QUndoCommand>

/**
 * @brief The UpdateScriptCommand class represents updating a @Script's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateScriptCommand : public QUndoCommand
{
public:
        UpdateScriptCommand(PropertiesPanel *panel, Script *script, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Script *m_script;           // The script which we are modifying parameters

	// Old parameters
	qreal m_oldTime;
	TimeUnit m_oldUnit;
	bool m_oldEncrypt;

	// New parameters
	qreal m_newTime;
	TimeUnit m_newUnit;
	bool m_newEncrypt;
};

#endif // UPDATESCRIPTCOMMAND_H
