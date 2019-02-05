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

	// New parameters (ones the box will have after updating its parameters)
	QString m_newTitle;
	int m_newRows;
	int m_newCols;
	bool m_newActivity;
	bool m_newPublish;
	QString m_newTopic;
};

#endif // UPDATEBOXCOMMAND_H
