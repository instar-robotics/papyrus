#include "constantdiagrambox.h"

ConstantDiagramBox::ConstantDiagramBox(const QString &name,
                                       const QIcon &icon,
                                       OutputSlot *outputSlot,
                                       const QUuid &uuid,
                                       QGraphicsItem *parent) :
    DiagramBox(name, icon, outputSlot, std::set<InputSlot *>(), uuid, parent)
{

}
