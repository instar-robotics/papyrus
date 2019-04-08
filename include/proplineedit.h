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

#ifndef PROPLINEEDIT_H
#define PROPLINEEDIT_H

#include <QLineEdit>

/**
 * @brief The PropLineEdit class is a standard @QLineEdit with only one minor modification: its
 * sizeHint can be changed
 */

class PropLineEdit : public QLineEdit
{
public:
	PropLineEdit(QWidget *parent = nullptr);
	PropLineEdit(const QString &contents, QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint; // Modified sizeHint
};

#endif // PROPLINEEDIT_H
