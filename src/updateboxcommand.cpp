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

#include "helpers.h"
#include "updateboxcommand.h"
#include "constantdiagrambox.h"

#include <QDebug>

UpdateBoxCommand::UpdateBoxCommand(PropertiesPanel *panel, DiagramBox *box, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_panel(panel),
      m_box(box)
{
	if (m_box == nullptr) {
		qWarning() << "[UpdateBoxCommand] created with null pointer in place of box!";
		return;
	}

	if (m_panel == nullptr) {
		qWarning() << "[UpdateBoxCommand] created with null pointer in place of panel!";
	}

	// Save the current parameter values
	m_oldTitle = m_box->title();
	m_oldRows = m_box->rows();
	m_oldCols = m_box->cols();
	m_oldActivity = m_box->saveActivity();
	m_oldPublish = m_box->publish();
	m_oldTopic = m_box->topic();

	// Save new parameter values
	m_newTitle = m_panel->boxTitle()->text();
	m_newRows = m_panel->rowsInput()->value();
	m_newCols = m_panel->colsInput()->value();
	m_newActivity = m_panel->saveActivity()->isChecked();
	m_newPublish = m_panel->publish()->isChecked();
	m_newTopic = m_panel->topic()->text();
}

void UpdateBoxCommand::undo()
{
	// Simply reset all parameters to their old values
	m_box->setTitle(m_oldTitle);
	m_box->setRows(m_oldRows);
	m_box->setCols(m_oldCols);
	m_box->setSaveActivity(m_oldActivity);
	m_box->setPublish(m_oldPublish);
	m_box->setTopic(m_oldTopic);

	m_box->update();

	// Also update all links from and to this box, because size, type, etc. might have been changed
	foreach (InputSlot *iSlot, m_box->inputSlots()) {
		foreach (Link *link, iSlot->inputs())
			link->update();
	}

	foreach (Link *link, m_box->outputSlot()->outputs())
		link->update();

	if (m_box->getScript() != nullptr)
		m_box->getScript()->setStatusModified(true);
}

void UpdateBoxCommand::redo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateBoxCommand] cannot undo: null pointer for panel!";
		return;
	}

	if (m_box == nullptr) {
		qWarning() << "[UpdateBoxCommand] cannot undo: null pointer for box!";
		return;
	}

	// Update the box's properties

	m_box->setTitle(m_newTitle);

	// If the box's output is matrix, then set its rows and cols
	if (m_box->outputType() == MATRIX) {
		// Check the shape of the matrix and decide whether it's valid
		bool okToUpdateSize = false;
		MatrixShape matrixShape = m_box->matrixShape();

		if (matrixShape == SHAPE_NONE)
			okToUpdateSize = true; // No restriction on size
		else if (matrixShape == VECT) {
			// For VECT shape, there must be at least one size of dimension 1
			if (m_panel->rowsInput()->value() != 1 && m_panel->colsInput()->value() != 1) {
				okToUpdateSize = false;
//				emit displayStatusMessage("VECT shape requires either rows = 1 or cols = 1", MSG_WARNING);
			} else {
				okToUpdateSize = true;
			}
		} else {
			// For the other types, the interface is grayed out, preventing user from changing values
			okToUpdateSize = true;
		}

		if (okToUpdateSize) {
			m_box->setRows(m_newRows);
			m_box->setCols(m_newCols);
		}

		m_box->updateSizeIcon();
	}

	// Set the box's "save activity" flag
	m_box->setSaveActivity(m_newActivity);

	// Set the box's publish and topic name (if it's valid)
	m_box->setPublish(m_newPublish);

	QString topic = m_newTopic;
	if (topic.isEmpty()) {
		Script *s = m_box->getScript();

		// Since this box is in a script, it should have a Script*
		if (s == nullptr) {
			informUserAndCrash(QObject::tr("Box in scene is lacking a Script!"));
		}
		m_box->setTopic(ensureSlashPrefix(mkTopicName(s->name(), m_box->uuid().toString())));
		m_panel->topic()->setStyleSheet("color: red;");
	} else if (isTopicNameValid(topic)) {
		m_box->setTopic(ensureSlashPrefix(topic));
		m_panel->topic()->setStyleSheet("color: black;"); // restore normal, black color
	} else
		m_panel->topic()->setStyleSheet("color: red;"); // mark invalid topic name in red (and don't update it)

	m_box->update();

	// Also update all links from and to this box, because size, type, etc. might have been changed
	foreach (InputSlot *iSlot, m_box->inputSlots()) {
		foreach (Link *link, iSlot->inputs())
			link->update();
	}

	foreach (Link *link, m_box->outputSlot()->outputs())
		link->update();

	if (m_box->getScript() != nullptr)
		m_box->getScript()->setStatusModified(true);
}
