#include "slot.h"

Slot::Slot(QGraphicsItem *parent) : QGraphicsItem(parent), m_dist(0), m_inputType(SCALAR_SCALAR)
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

qreal Slot::dist() const
{
    return m_dist;
}

void Slot::setDist(const qreal &dist)
{
    m_dist = dist;
}

DiagramBox *Slot::box() const
{
    return m_box;
}

void Slot::setBox(DiagramBox *box)
{
    m_box = box;
}

InputType Slot::inputType() const
{
    return m_inputType;
}

void Slot::setInputType(const InputType &type)
{
    m_inputType = type;
}
