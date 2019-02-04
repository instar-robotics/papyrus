#ifndef ADDBOXCOMMAND_H
#define ADDBOXCOMMAND_H

#include "diagrambox.h"
#include "diagramscene.h"

#include <QUndoCommand>

/**
 * @brief The AddBoxCommand class represents adding a @DiagramBox to aa @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddBoxCommand : public QUndoCommand
{
public:
	AddBoxCommand(DiagramScene *scene, DiagramBox *box, const QPointF &initialPos, QUndoCommand *parent = nullptr);
	~AddBoxCommand();

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene; // The scene in which the box should be added
	DiagramBox *m_box;     // The DiagramBox to add to the scene
	QPointF m_initialPos;  // The initial position at which the box is added
};

#endif // ADDBOXCOMMAND_H
