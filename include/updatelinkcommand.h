#ifndef UPDATELINKCOMMAND_H
#define UPDATELINKCOMMAND_H

#include "propertiespanel.h"
#include "link.h"

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

	// New parameters
	qreal m_newWeight;
	QString m_newValue;
	bool m_newSecondary;
};

#endif // UPDATELINKCOMMAND_H
