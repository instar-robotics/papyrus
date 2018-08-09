#ifndef CATEGORY_H
#define CATEGORY_H

#include "function.h"

#include <vector>
#include <QTreeWidgetItem>

/**
 * @brief The Category class holds a number of neural @Function 's descriptions that are part of the
 * same category.
 * This is used with the @Library and the @LibraryPanel. The idea is to group similar functions
 * by themes. The categories are simply made by parsing the directory name in which the function's
 * descriptions are saved.
 */

class Category : public QTreeWidgetItem
{
public:
    Category(const QString &name);

    QString name() const;
    void setName(const QString &name);

private:
    QString m_name;
};

#endif // CATEGORY_H
