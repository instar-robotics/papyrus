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

#ifndef LIBRARYPANEL_H
#define LIBRARYPANEL_H

#include <QTreeWidget>

/**
 * @brief The LibraryPanel class is the widget that goes in the left toolbar. It will hold
 * @Function s through the @Library.
 */
class LibraryPanel : public QTreeWidget
{
    Q_OBJECT

public:
    explicit LibraryPanel(QWidget *parent = 0);

    static QString libraryItemMimeType() { return QStringLiteral("application/x-neural-box"); }

protected:
    void dragEnterEvent(QDragEnterEvent *evt) override;
    void dragMoveEvent(QDragMoveEvent *evt) override;
    void dropEvent(QDropEvent *evt) override;
    void startDrag(Qt::DropActions supportedActions) override;
};

#endif // LIBRARYPANEL_H
