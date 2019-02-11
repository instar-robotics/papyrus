#include "helpers.h"
#include "updatelinkcommand.h"

#include <QDebug>

UpdateLinkCommand::UpdateLinkCommand(PropertiesPanel *panel, Link *link, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_panel(panel),
      m_link(link)
{
	if (m_link == nullptr) {
		qWarning() << "[UpdateLinkCommand] created with null pointer in place of link!";
		return;
	}

	if (m_panel == nullptr) {
		qWarning() << "[UpdateLinkCommand] created with null pointer in place of panel!";
	}

	// Save the current parameter values
	m_oldWeight = m_link->weight();
	m_oldValue = m_link->value();
	m_oldSecondary = m_link->secondary();

	// Save new parameter values
	m_newWeight = m_panel->linkWeight()->value();
	m_newValue = m_panel->linkValue()->text();
	m_newSecondary = m_panel->linkSecondary()->isChecked();
}

void UpdateLinkCommand::undo()
{
	if (m_link->isStringLink())
		m_link->setValue(m_oldValue);
	else
		m_link->setWeight(m_oldWeight);

	m_link->setSecondary(m_oldSecondary);

	m_link->update();

	if (m_link->to() != nullptr && m_link->to()->scene() != nullptr) {
		DiagramScene *dScene = dynamic_cast<DiagramScene *>(m_link->to()->scene());

		if (dScene != nullptr && dScene->script() != nullptr)
			dScene->script()->setStatusModified(true);
	}
}

void UpdateLinkCommand::redo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateLinkCommand] cannot undo: null pointer for panel!";
		return;
	}

	if (m_link == nullptr) {
		qWarning() << "[UpdateLinkCommand] cannot undo: null pointer for link!";
		return;
	}

	if (m_link->isStringLink())
		m_link->setValue(m_newValue);
	else
		m_link->setWeight(m_newWeight);

	m_link->setSecondary(m_newSecondary);

	m_link->update();

	if (m_link->to() != nullptr && m_link->to()->scene() != nullptr) {
		DiagramScene *dScene = dynamic_cast<DiagramScene *>(m_link->to()->scene());

		if (dScene != nullptr && dScene->script() != nullptr)
			dScene->script()->setStatusModified(true);
	}
}