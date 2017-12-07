#ifndef LIBRARY_H
#define LIBRARY_H

#include "category.h"

#include <vector>

/**
 * @brief The Library class holds the categories which in turn hold all the functions
 * that have a valid XML description file. These functions are then used as building blocks for
 * creating neural programs.
 */

class Library
{
public:
    Library();

    void addCategory(Category *category);

    std::vector<Category *> categories() const;
    void setCategories(const std::vector<Category *> &categories);

private:
    std::vector<Category *> m_categories;
};

#endif // LIBRARY_H
