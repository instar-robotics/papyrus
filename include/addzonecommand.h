#ifndef ADDZONECOMMAND_H
#define ADDZONECOMMAND_H

#include "diagramscene.h"
#include "zone.h"

#include <QUndoCommand>

/**
 * @brief The AddZoneCommand class represents adding a @Zone to aa @DiagramScene. This is used
 * to provide Undo/Redo functionality.
 */
class AddZoneCommand : public QUndoCommand
{
public:
	AddZoneCommand(DiagramScene *scene, Zone *zone, QUndoCommand *parent = nullptr);

	void undo() override;
	void redo() override;
private:
	DiagramScene *m_scene;  // The scene in which to add the zone
	Zone *m_zone;           // The zone to add to the scene
};

#endif // ADDZONECOMMAND_H
