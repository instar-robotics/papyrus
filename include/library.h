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
	~Library();

	void addCategory(Category *category);

	std::vector<Category *> categories() const;
	void setCategories(const std::vector<Category *> &categories);

private:
	std::vector<Category *> m_categories;
};

#endif // LIBRARY_H
