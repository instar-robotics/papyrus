#include "helpers.h"
#include "updatescriptcommand.h"

#include <QDebug>

UpdateScriptCommand::UpdateScriptCommand(PropertiesPanel *panel, Script *script, QUndoCommand *parent)
    : QUndoCommand(parent),
      m_panel(panel),
      m_script(script)
{
	if (m_script == nullptr) {
		qWarning() << "[UpdateScriptCommand] created with null pointer in place of script!";
		return;
	}

	if (m_panel == nullptr) {
		qWarning() << "[UpdateScriptCommand] created with null pointer in place of panel!";
	}

	// Save the current parameter values
	m_oldTime = script->timeValue();
	m_oldUnit = script->timeUnit();
	m_oldEncrypt = script->encrypt();

	// Save new parameter values
	m_newTime = m_panel->timeValue()->value();
	m_newUnit = m_panel->timeUnit()->currentData().value<TimeUnit>();
	m_newEncrypt = m_panel->encrypt()->isChecked();
}

void UpdateScriptCommand::undo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateScriptCommand] cannot undo: null pointer for panel!";
		return;
	}

	if (m_script == nullptr) {
		qWarning() << "[UpdateScriptCommand] cannot undo: null pointer for script!";
		return;
	}

	m_script->setTimeValue(m_oldTime);
	m_script->setTimeUnit(m_oldUnit);
	m_script->setEncrypt(m_oldEncrypt);

	m_script->setStatusModified(true);
}

void UpdateScriptCommand::redo()
{
	if (m_panel == nullptr) {
		qWarning() << "[UpdateScriptCommand] cannot undo: null pointer for panel!";
		return;
	}

	if (m_script == nullptr) {
		qWarning() << "[UpdateScriptCommand] cannot undo: null pointer for script!";
		return;
	}

	m_script->setTimeValue(m_newTime);
	m_script->setTimeUnit(m_newUnit);
	m_script->setEncrypt(m_newEncrypt);

	m_script->setStatusModified(true);
}
