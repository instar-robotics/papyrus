#include "slot.h"

Slot::Slot(QString &name, QGraphicsItem *parent) : QGraphicsItem(parent),
                                                   m_name(name)
{

}

QString Slot::name() const
{
    return m_name;
}

