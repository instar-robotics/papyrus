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

#ifndef UPDATELINKCOMMAND_H
#define UPDATELINKCOMMAND_H

#include "propertiespanel.h"
#include "link.h"
#include "types.h"

#include <QUndoCommand>

/**
 * @brief The UpdateLinkCommand class represents updating a @Link's properties through the
 * @PropertiesPanel. This is used to provide Undo/Redo functionality.
 */
class UpdateLinkCommand : public QUndoCommand
{
public:
	UpdateLinkCommand(PropertiesPanel *panel, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	PropertiesPanel *m_panel; // The properties panel which contains the fields with the new values
	Link *m_link;             // The link which we are modifying parameters

	// Old parameters
	qreal m_oldWeight;
	QString m_oldValue;
	bool m_oldSecondary;
	Connectivity m_oldConnectivity;
	QString m_oldRegexes;
	bool m_oldUseValue;

	// New parameters
	qreal m_newWeight;
	QString m_newValue;
	bool m_newSecondary;
	Connectivity m_newConnectivity;
	QString m_newRegexes;
	bool m_newUseValue;
};

#endif // UPDATELINKCOMMAND_H
