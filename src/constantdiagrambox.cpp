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

#include "constantdiagrambox.h"

#include <QDebug>
#include <QStyleOptionGraphicsItem>

ConstantDiagramBox::ConstantDiagramBox(const QString &name,
                                       OutputSlot *outputSlot,
                                       const QUuid &uuid,
                                       QGraphicsItem *parent) :
    DiagramBox(name, outputSlot, std::vector<InputSlot *>(), uuid, nullptr, parent)
{
	m_bWidth = 70;

	// We need to re-call this, because it was called in the parent's constructor, and the box's
	// width was not changed yet
	setOutputSlotPos();
}

void ConstantDiagramBox::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(widget);

	QPen pen;
	qreal width = 1.5;

	QFont font = painter->font();
	font.setPixelSize(13);

	// If the box is selected, make it appear bold
	if (option->state & QStyle::State_Selected) {
		width += 1;
		font.setBold(true);
	}

	pen.setWidthF(width);
	painter->setFont(font);

	QColor color = Qt::gray;
	color = color.dark();

	pen.setColor(color);

	painter->setPen(pen);

	// We have to calculate offsets to draw lines in order to stay inside the boundingRect
	qreal x0 = 0 + width / 2.0;
	qreal y0 = x0;
	qreal w = m_bWidth - width; // width/2 on the left and width/2 on the right, hence width
	qreal h = m_bHeight - width;

	// Draw enclosure
	painter->drawRoundedRect(QRectF(x0, y0, w, h), 5, 5);
	// Fill enclosure (use a QPainterPath because there's no fillRoundedRect()
	QPainterPath encPath;
	encPath.addRoundedRect(QRectF(x0, y0, w, h), 5, 5);
	painter->fillPath(encPath, Qt::white);

	// Draw horizontal line to create the space for the function's name, with dashed line
	pen.setStyle(Qt::DotLine);
	painter->setPen(pen);

	// width and not 1.5 * width at the end for aesthetics (dots don't go toward the end of line)
	painter->drawLine(QLineF(1.5 * width, m_bHeight - m_tHeight, m_bWidth - width, m_bHeight - m_tHeight));

	pen.setStyle(Qt::SolidLine);
	painter->setPen(pen);

	// The function's icon is not drawn here since it's a SVG element set as a child of this one

	// Draw the function's name
	painter->drawText(QRectF(0, m_bHeight - m_tHeight, m_bWidth, m_tHeight), Qt::AlignCenter, m_name);
}
