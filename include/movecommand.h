#ifndef MOVECOMMAN_H
#define MOVECOMMAN_H

#include "diagrambox.h"

#include <QUndoCommand>

/**
 * @brief The MoveCommand class represents moving a @DiagramBox on a @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */

class MoveCommand : public QUndoCommand
{
public:
	MoveCommand(DiagramBox *box, const QPointF &oldPos, QUndoCommand *parent = nullptr);
	void undo() override;
	void redo() override;

private:
	DiagramBox *m_box;
	QPointF m_oldPos;
	QPointF m_newPos;
};

#endif // MOVECOMMAN_H
