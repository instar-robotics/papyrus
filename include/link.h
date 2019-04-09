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

#ifndef LINK_H
#define LINK_H

#include "script.h"
#include "types.h"

#include <QGraphicsItem>
#include <QUuid>
#include <QPainterPath>
#include <QGraphicsLineItem>
#include <QString>

/**
 * @brief The Link class represents a link between neural functions (more precisely between
 * @DiagramBox es) Even more precisely, a Link is between a @DiagramBox 's @OutputSlot and
 * another (or same) @DiagramBox 's @InputSlot.
 * Depending on its type, it can have a weight.
 */

class InputSlot;
class OutputSlot;

Q_DECLARE_METATYPE(Connectivity);

class Link : public QObject, public QGraphicsItem
{
	Q_OBJECT

public:
	explicit Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent = 0);

	QRectF boundingRect() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
	QPainterPath shape() const;
	bool isStringLink();
	void updateTooltip();

	void addLinesToScene();
	void removeLinesFromScene();

	void updateLines();

	bool checkIfInvalid();

	QUuid uuid() const;
	void setUuid(const QUuid &uuid);

	OutputSlot *from() const;
	void setFrom(OutputSlot *from);

	InputSlot *to() const;
	void setTo(InputSlot *to);

	bool secondary() const;
	void setSecondary(bool secondary);

	qreal weight() const;
	void setWeight(const qreal &weight);

	bool isInvalid() const;
	void setIsInvalid(bool isInvalid);

	bool selfLoop() const;

	QString value() const;
	void setValue(const QString &value);

	Connectivity connectivity() const;
	void setConnectivity(const Connectivity &connectivity);

	LinkInvalidReason invalidReason() const;
	void setInvalidReason(const LinkInvalidReason &invalidReason);

private:
	bool checkIfSelfLoop();

	QUuid m_uuid;           // Unique identifier
	OutputSlot *m_from;     // The OutputSlot this link goes from
	InputSlot *m_to;        // The InputSlot this link goes to
	bool m_secondary;       // Tells whether a link is a secondary link
	bool m_selfLoop;        // Tells whether a link loop back to the same function

	QGraphicsLineItem m_line;          // Main line that represents the link
	QGraphicsLineItem m_rightSegment;  // Right segment (for secondary links)
	QGraphicsLineItem m_leftSegment;   // Left segment (for secondary links)

	qreal m_weight;            // The weight associated to this link
	QString m_value;           // The string value associated to this link (when between strings)

	bool m_isInvalid; // Tells that this link is currently not valid (error in type, in sizes, etc.)
	LinkInvalidReason m_invalidReason; // Tell why a link is invalid

	Connectivity m_connectivity; // Only viable for MATRIX_MATRIX
};

#endif // LINK_H
