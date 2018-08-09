#include "category.h"

Category::Category(const QString &name) : QTreeWidgetItem(), m_name(name)
{
    setExpanded(false); // By default, collapse all categories
}

QString Category::name() const
{
    return m_name;
}

void Category::setName(const QString &name)
{
    m_name = name;
}

