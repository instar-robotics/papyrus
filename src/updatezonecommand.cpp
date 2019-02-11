#include "helpers.h"
#include "updatezonecommand.h"

#include <QDebug>

UpdateZoneCommand::UpdateZoneCommand(PropertiesPanel *panel, Zone *zone, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_panel(panel),
      m_zone(zone)
{
	if (m_zone == nullptr) {
		qWarning() << "[UpdateZoneCommand] created with null pointer in place of zone!";
		return;
	}

	if (m_panel == nullptr) {
		qWarning() << "[UpdateZoneCommand] created with null pointer in place of panel!";
	}

	// Save the current parameter values
	m_oldTitle = m_zone->title();
	m_oldColor = m_zone->color();

	// Save new parameter values
	m_newTitle = m_panel->zoneTitle()->text();
	m_newColor = m_panel->zoneColor()->color();
}

void UpdateZoneCommand::undo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot undo: null pointer for panel!";
		return;
	}

	if (m_zone == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot undo: null pointer for zone!";
		return;
	}

	m_zone->setTitle(m_oldTitle);
	m_zone->setColor(m_oldColor);

	m_zone->update();

	if (m_zone->scene() != nullptr) {
		DiagramScene *dScene = dynamic_cast<DiagramScene *>(m_zone->scene());

		if (dScene != nullptr && dScene->script() != nullptr)
			dScene->script()->setStatusModified(true);
	}
}

void UpdateZoneCommand::redo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot redo: null pointer for panel!";
		return;
	}

	if (m_zone == nullptr) {
		qWarning() << "[UpdateZoneCommand] cannot redo: null pointer for zone!";
		return;
	}

	m_zone->setTitle(m_newTitle);
	m_zone->setColor(m_newColor);

	m_zone->update();

	if (m_zone->scene() != nullptr) {
		DiagramScene *dScene = dynamic_cast<DiagramScene *>(m_zone->scene());

		if (dScene != nullptr && dScene->script() != nullptr)
			dScene->script()->setStatusModified(true);
	}
}
