#include "library.h"

Library::Library()
{

}

void Library::addCategory(Category *category)
{
    m_categories.push_back(category);
}

std::vector<Category *> Library::categories() const
{
    return m_categories;
}

void Library::setCategories(const std::vector<Category *> &categories)
{
    m_categories = categories;
}

