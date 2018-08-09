#include "category.h"

Category::Category(const QString &name) : QTreeWidgetItem(), m_name(name)
{

}

/*
void Category::addFunction(Function *function)
{
    m_functions.push_back(function);
}
//*/

QString Category::name() const
{
    return m_name;
}

void Category::setName(const QString &name)
{
    m_name = name;
}

/*
std::vector<Function *> Category::functions() const
{
    return m_functions;
}
//*/
