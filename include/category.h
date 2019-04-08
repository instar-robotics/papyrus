/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER
 
  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.
 
  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

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
