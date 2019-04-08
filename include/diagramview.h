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

#ifndef DIAGRAMVIEW_H
#define DIAGRAMVIEW_H

#include <QGraphicsView>

/**
 * @brief The DiagramView class is used to render a @DiagramScene (the @DiagramScene holds
 * the data: the items added, their positions, etc.) and the DiagramView displays it. This
 * separation by Qt allows several views to be attached to a single scene (we will use
 * something like this when we implement the minimap).
 */
class DiagramView : public QGraphicsView
{
	Q_OBJECT
public:
	explicit DiagramView(QWidget *parent = 0);
	DiagramView(QGraphicsScene *scene, QWidget *parent = 0);

signals:

protected:
	void wheelEvent(QWheelEvent *evt);

public slots:
};

#endif // DIAGRAMVIEW_H
