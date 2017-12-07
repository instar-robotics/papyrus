#ifndef CATEGORY_H
#define CATEGORY_H

#include "function.h"

#include <vector>
#include <QTreeWidgetItem>

/**
 * @brief The Category class holds a number of neural function's descriptions that are part of the
 * same category.
 */

class Category : public QTreeWidgetItem
{
public:
    Category(QString &name);

//    void addFunction(Function *function);
    QString name() const;
    void setName(const QString &name);

//    std::vector<Function *> functions() const;

private:
    QString m_name;
//    std::vector<Function *> m_functions;
};

#endif // CATEGORY_H
