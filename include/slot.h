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

#ifndef SLOT_H
#define SLOT_H

#include <QObject>
#include <QString>
#include <QGraphicsItem>

//#include <diagrambox.h>
class DiagramBox;

/**
 * @brief The Slot class defines an argument slot, i.e. an item will either receive
 * a connection from another box's output slot, or from which a connection leaves to
 * reach another box' input slot.
 * This class is meant to be subclassed (see @InputSlot and @OutputSlot).
 */

/*
enum InputType {
	SCALAR_SCALAR,
	SCALAR_MATRIX,
	MATRIX_MATRIX,
	SPARSE_MATRIX
};

enum OutputType {
	SCALAR,
	MATRIX
};
*/

class Slot : public QObject, public QGraphicsItem
{
	Q_OBJECT
public:
	explicit Slot(QGraphicsItem *parent = nullptr);
	explicit Slot(QString &name, QGraphicsItem *parent = nullptr);

	QString name() const;
	void setName(const QString &name);

	virtual QRectF boundingRect() const = 0;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) = 0;

	qreal dist() const;
	void setDist(const qreal &dist);

	DiagramBox *box() const;
	void setBox(DiagramBox *box);

	qreal radius() const;

protected:
	QString m_name;    // The name of this slot
	qreal m_dist;      // Distance to the mouse (used to highlight the slot when mouse approach)
	DiagramBox *m_box; // The DiagramBox that is associated with this Slot
	qreal m_radius;    // base radius for drawing the slot
};

#endif // SLOT_H
