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

#ifndef OUTPUTSLOT_H
#define OUTPUTSLOT_H

#include "slot.h"
#include "types.h"

#include <QString>
#include <set>

/**
 * @brief The specialized version of @Slot used for output.
 * They contain @Link s that goes out of a @DiagramBox.
 * Outputs slots react to the mouse by growing sightly when the mouse comes nears them, in order
 * to facilitate the creation of a link.
 */

class Link;

class OutputSlot : public Slot
{
    Q_OBJECT
public:
    explicit OutputSlot();

    std::set<Link *> outputs() const;

    void addOutput(Link *output);
    void removeOutput(Link *output);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const override;

    void mousePressEvent(QGraphicsSceneMouseEvent *evt);
    void hoverEnterEvent(QGraphicsSceneHoverEvent *evt);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *evt);

    bool isDrawingLine() const;
    void setIsDrawingLine(bool isDrawingLine);

    void updateLinks();

    OutputType outputType() const;
    void setOutputType(const OutputType &outputType);

private:
    std::set<Link *> m_outputs; // The set of links which leaves this slot
    bool m_isDrawingLine;       // Indicate if we are drawing an outgoing link
    OutputType m_outputType;    // Indicate whether this function (slot) outputs a matrix or scalar
};

#endif // OUTPUTSLOT_H
