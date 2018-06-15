#ifndef LIBRARY_H
#define LIBRARY_H

#include "category.h"

#include <vector>

/**
 * @brief The Library class holds the @Categorie s which in turn hold all the @Function s
 * that have a valid XML description file. These @Function are then used as building blocks for
 * creating neural programs (by converting them to @DiagramBox es).
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
