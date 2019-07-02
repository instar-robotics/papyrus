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

#ifndef INPUTSLOT_H
#define INPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>
#include <QGraphicsSimpleTextItem>
#include <QUuid>

class Link;

/**
 * @brief The InputSlot class is attached to a @DiagramBox. This is represented as the input
 * little circle to a @DiagramBox. Its job is to keep track of all @Link s from other
 * @Diagrambox es.
 * There is one @InputSlot per <input> in the function's XML description file.
 */
class InputSlot : public Slot
{
	Q_OBJECT

public:
	explicit InputSlot();
	explicit InputSlot(const QString &name);
	explicit InputSlot(const InputSlot &copy);

	bool multiple() const;
	void setMultiple(bool allowMultiple);

	std::vector<Link *> inputs() const;

	void addInput(Link *input, bool ignoreFull = false);
	void removeInput(Link *input);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	QRectF boundingRect() const override;

	void updateLinks();

	InputType inputType() const;
	void setInputType(const InputType &inputType);

	bool canLink() const;
	void setCanLink(bool canLink);

	const QGraphicsSimpleTextItem &label();

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	bool checkSize() const;
	void setCheckSize(bool checkSize);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

protected:
	QUuid m_uuid;              // Unid ID for the slot (used for kheops)
	bool m_multiple;           // Whether this slot can receive several links
	std::vector<Link *> m_inputs; // The set of links connected to this slot
	InputType m_inputType;     // Indicate type and connectivity of this input
	bool m_canLink;            // Indicate if this input can be linked to the current output when creating a Link
	QGraphicsSimpleTextItem m_label; // A label that contains this input's name
	bool m_checkSize;          // Whether or not this input must check that sizes are correct (on SCALAR_MATRIX)
	QString m_description;     // Description of the input
	MatrixShape m_matrixShape; // The shape of the matrix it can accept

signals:
	void slotFull(); // Fired when trying to add a second input to a slot that doesn't allow multiple
};

#endif // INPUTSLOT_H
