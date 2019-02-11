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
