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
