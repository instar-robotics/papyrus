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

#ifndef ZONE_H
#define ZONE_H

#include <QGraphicsObject>
#include <QGraphicsItem>
#include <QColor>
#include <QGraphicsItemGroup>

enum ResizeType {
	NO_RESIZE,
	RESIZE_TOP,
	RESIZE_RIGHT,
	RESIZE_BOTTOM,
	RESIZE_LEFT
};

class Zone : public QGraphicsObject
{
public:
	explicit Zone(QGraphicsObject *parent = nullptr);
	explicit Zone(qreal x, qreal y, qreal w, qreal h, QGraphicsObject *parent = nullptr);
	~Zone();

	QRectF boundingRect() const override;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QVariant itemChange(GraphicsItemChange change, const QVariant &value);
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void hoverMoveEvent(QGraphicsSceneHoverEvent *event);

	void updateGroup(bool startFromScratch = false);
	void updateLinks();

	// Getters / Setters

	qreal width() const;
	void setWidth(const qreal &width);

	qreal height() const;
	void setHeight(const qreal &height);

	QColor color() const;
	void setColor(const QColor &color);

	QString title() const;
	void setTitle(const QString &title);

private:
	qreal m_width;
	qreal m_height;
	QColor m_color;
	QString m_title; // The title of the comment zone (should be kept small)
	ResizeType m_resizeType;
};

#endif // ZONE_H
