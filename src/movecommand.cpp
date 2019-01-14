#include "movecommand.h"

#include <QDebug>


MoveCommand::MoveCommand(DiagramBox *box, const QPointF &oldPos, QUndoCommand *parent)
 : QUndoCommand(parent),
   m_box(box),
   m_oldPos(oldPos)
{
	if (box == nullptr) {
		qWarning() << "[MoveCommand] created with a null pointer in place of box!";
		return;
	}

	qDebug() << "[MoveCommand] created";

	m_newPos = box->scenePos();
}

void MoveCommand::undo()
{
	if (m_box == nullptr) {
		qWarning() << "[MoveCommand] cannot undo operation: box is null!";
		return;
	}

	qDebug() << "[MoveCommand] undo()";
	m_box->setPos(m_oldPos);

	if (m_box->scene() == nullptr) {
		qWarning() << "[MoveCommand] cannot undo operation: box's scene is null!";
		return;
	}

	m_box->scene()->update();
	// Dirty trick to trigger the itemPositionChange event and have links updated
	m_box->moveBy(0, 0);
	m_box->moveBy(-0.1,0);
}

void MoveCommand::redo()
{
	if (m_box == nullptr) {
		qWarning() << "[MoveCommand] cannot redo operation: box is null!";
		return;
	}

	qDebug() << "[MoveCommand] redo()";
	m_box->setPos(m_newPos);

	if (m_box->scene() == nullptr) {
		qWarning() << "[MoveCommand] cannot redo operation: box's scene is null!";
		return;
	}

	m_box->scene()->update();
	// Dirty trick to trigger the itemPositionChange event and have links updated
	m_box->moveBy(0.1, 0);
	m_box->moveBy(-0.1,0);
}
