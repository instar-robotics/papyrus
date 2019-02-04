#ifndef ADDLINKCOMMAND_H
#define ADDLINKCOMMAND_H

#include "diagramscene.h"
#include "link.h"
#include "inputslot.h"
#include "outputslot.h"

#include <QUndoCommand>

/**
 * @brief The AddLinkCommand class represents adding a @Link to a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddLinkCommand : public QUndoCommand
{
public:
	AddLinkCommand(DiagramScene *scene, Link *link, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the link is added
	Link *m_link;          // The link to add to the scene
	OutputSlot *m_from;    // The output slot the link is from
	InputSlot *m_to;       // The input slot the link goes to
	bool m_isFirst;        // Flag that indicates wheather a redo() action is the first one
};

#endif // ADDLINKCOMMAND_H
