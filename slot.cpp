#include "slot.h"

Slot::Slot(QGraphicsItem *parent) : QGraphicsItem(parent)
{
    m_name.clear();
}

Slot::Slot(QString &name, QGraphicsItem *parent) : Slot(parent)
{
    setName(name);
}

Slot::~Slot()
{

}

QString Slot::name() const
{
    return m_name;
}

void Slot::setName(const QString &name)
{
    m_name = name;
}
