#ifndef SWAPBOXESCOMMAND_H
#define SWAPBOXESCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"
#include "link.h"

#include <QUndoCommand>

/**
 * @brief The SwapBoxesCommand class represents swapping two functions. This is used to provide
 * Undo/Redo functionality.
 */

class SwapBoxesCommand : public QUndoCommand
{
public:
	SwapBoxesCommand(DiagramScene *scene, DiagramBox *toSwap, DiagramBox *newBox,
	                 QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;

private:
	DiagramScene *m_scene;  // The scene in which the swap takes place
	DiagramBox *m_toSwap;   // The box to be swapped
	DiagramBox *m_newBox;   // The box to add

	QList<Link *> m_outputLinks; // List of all links that are FROM m_toSwap
	QHash<QString, QList<Link *>> m_inputLinks; // List of all links that are TO m_toSwap, sorted by input slots
};

#endif // SWAPBOXESCOMMAND_H
