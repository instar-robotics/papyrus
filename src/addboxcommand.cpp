#include "addboxcommand.h"

#include <QDebug>

AddBoxCommand::AddBoxCommand(DiagramScene *scene, DiagramBox *box, const QPointF &initialPos,
                             QUndoCommand *parent)
    : QUndoCommand(parent),
      m_scene(scene),
      m_box(box),
      m_initialPos(initialPos)
{
	if (box == nullptr) {
		qWarning() << "[AddBoxCommand] created with a null pointer in place of box!";
		return;
	}

	if (scene == nullptr) {
		qWarning() << "[AddBoxCommand] created with a null pointer in place of scene!";
		return;
	}
}

AddBoxCommand::~AddBoxCommand()
{
	// If we have a box in the pointer and this box is not in the scene, we need to destroy it,
	// because we are the only one with this ressource
	qWarning() << "[AddBoxCommand] destructor should check if it should delete the box or not!";
}

void AddBoxCommand::undo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddBoxCommand] cannot undo: null pointer for scene!";
		return;
	}

	// do not use deleteItem() because we only want to remove it from the scene, not destroy it
	m_scene->removeItem(m_box);
	m_scene->update();
}

void AddBoxCommand::redo()
{
	if (m_scene == nullptr) {
		qWarning() << "[AddBoxCommand] cannot redo: null pointer for scene!";
		return;
	}

	m_scene->addBox(m_box, m_initialPos);
}
