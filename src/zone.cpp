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

#include "zone.h"
#include "diagrambox.h"
#include "constants.h"
#include "diagramscene.h"
#include "helpers.h"

#include <QDebug>
#include <QPen>
#include <QBrush>
#include <QPainter>
#include <QPainterPath>
#include <QGraphicsScene>
#include <QGraphicsSceneHoverEvent>

Zone::Zone(qreal x, qreal y, qreal w, qreal h, QGraphicsRectItem *parent)
    : QGraphicsRectItem(0, 0, w, h, parent),
      m_color(qRgba(51, 153, 255, 10)),
      m_resizeType(NO_RESIZE)
{
	m_color.setAlpha(80);

	setPos(x, y);

	setFlags(QGraphicsItem::ItemIsSelectable
	         | QGraphicsItem::ItemIsMovable
	         | QGraphicsItem::ItemSendsScenePositionChanges);
	setAcceptHoverEvents(true);

	setHandlesChildEvents(false); // so that we can still move boxes individually

	setZValue(COMMENTS_Z_VALUE);
}

Zone::Zone(QGraphicsRectItem *parent) : Zone(0,0,100,100,parent)
{

}

void Zone::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(widget);
	Q_UNUSED(option);

	// No simple function painter->fillRoundedRect(), so we must use a QPainterPath for this
	QPainterPath path;
	path.addRoundedRect(boundingRect(), 4, 4);
	painter->fillPath(path, m_color);

	// Draw the title of the comment zone
	QPointF txtOrigin = boundingRect().topLeft();
	txtOrigin.rx() += 10;
	txtOrigin.ry() += 20;
	QFont titleFont("Inconsolata");
	painter->setFont(titleFont);
	painter->drawText(txtOrigin, m_title);
}

/**
 * @brief Zone::mousePressEvent activates the correct resizing mode (if any) based on the position
 * of the mouse when clicked.
 * @param event
 */
void Zone::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	QPointF p(event->pos());


	// Flag resizing
	qreal margin = 10;
	if (p.x() <= margin) {
		m_resizeType = RESIZE_LEFT;
	} else if (p.x() >= rect().width() - margin) {
		m_resizeType = RESIZE_RIGHT;
	} else if (p.y() <= margin) {
		m_resizeType = RESIZE_TOP;
	} else if(p.y() > rect().height() - margin) {
		m_resizeType = RESIZE_BOTTOM ;
	} else {
		m_resizeType = NO_RESIZE;
	}

	QGraphicsRectItem::mousePressEvent(event);
}

/**
 * @brief Zone::mouseMoveEvent is used to grow or shrink the group's size if we are in a resizing
 * mode.
 * @param event
 */
void Zone::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	QPointF sPos, p;
	qreal dx, dy;

	// Get coordinate of the target new position
	QPointF targetPos = event->pos();

	// Get the scene in order to get the grid size
	DiagramScene *theScene = dynamic_cast<DiagramScene *>(scene());
	if (theScene == nullptr) {
		informUserAndCrash("Could not cast the scene into a DiagramScene!");
	}
	int gridSize = theScene->gridSize();

	// Snap the new position's (x, y) coordinates to the grid
	qreal newX = round(targetPos.x() / gridSize) * gridSize;
	qreal newY = round(targetPos.y() / gridSize) * gridSize;

	QRectF currRect;
	switch (m_resizeType) {
		// Resizing left can be done just by adjusting bottom right point
		case RESIZE_RIGHT:
			currRect = rect();
			currRect.setWidth(newX);
			setRect(currRect);
			update();
			theScene->update(); // COSTLY: TODO: find a better way
		break;

		case RESIZE_BOTTOM:
			currRect = rect();
			currRect.setHeight(newY);
			setRect(currRect);
			update();
			theScene->update(); // COSTLY: TODO: find a better way
		break;

		case RESIZE_TOP:
			sPos = event->scenePos();
			p = scenePos();
			dy = round((p.y() - sPos.y()) / gridSize) * gridSize;
			p.setY(sPos.y());
			setPos(p);
			currRect = rect();
			currRect.setHeight(currRect.height() + dy);
			setRect(currRect);

			// Move all children by the dy (because otherwise they move WITH the zone)
			foreach (QGraphicsItem *child, childItems()) {
				child->moveBy(0, dy);
			}

			update();
		break;

		case RESIZE_LEFT:
			sPos = event->scenePos();
			p = scenePos();
			dx = round((p.x() - sPos.x()) / gridSize) * gridSize;
			p.setX(sPos.x());
			setPos(p);
			currRect = rect();
			currRect.setWidth(currRect.width() + dx);
			setRect(currRect);

			// Move all children by the dx (because otherwise they move WITH the zone)
			foreach (QGraphicsItem *child, childItems()) {
				child->moveBy(dx, 0);
			}

			update();
		break;

		default:
			QGraphicsRectItem::mouseMoveEvent(event);
	}
}

/**
 * @brief Zone::mouseReleaseEvent is used to stop resizing when user releases its mouse.
 * @param event
 */
void Zone::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	// Update the group only when the Zone was resized
	if (m_resizeType != NO_RESIZE)
		updateGroup();

	m_resizeType = NO_RESIZE;

	QGraphicsRectItem::mouseReleaseEvent(event);
}

/**
 * @brief Zone::hoverMoveEvent changes the cursor shape according to the position inside the group
 * to hint the user he can resize.
 * @param event
 */
void Zone::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
	QPointF p = event->pos();
	qreal margin = 10;

	if (p.x() <= margin) {
		setCursor(Qt::SizeHorCursor);
	} else if (p.x() >= rect().width() - margin) {
		setCursor(Qt::SizeHorCursor);
	} else if (p.y() <= margin) {
		setCursor(Qt::SizeVerCursor);
	} else if(p.y() > rect().height() - margin) {
		setCursor(Qt::SizeVerCursor);
	} else {
		setCursor(Qt::ArrowCursor);
	}

	QGraphicsRectItem::hoverMoveEvent(event);
}

/**
 * @brief Zone::updateGroup makes sure that all @DiagramBox es that are colliding with this zone
 * are made children of it.
 */
void Zone::updateGroup()
{
	// Note: the position (pos()) of an item is based on its parent (or the scene if no parent)
	// this is why we have to use the sceneTransform

	// First check if all items are still inside the zone
	foreach (QGraphicsItem *item, childItems()) {
		if (!item->collidesWithItem(this)) {
			QPointF sP = item->scenePos();
			item->setParentItem(nullptr);
			item->setPos(sP);
		}
	}

	// Then check all colliding items, and add them inside the zone
	foreach (QGraphicsItem *item, collidingItems()) {
		// filter by box
		DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(item);
		if (maybeBox != nullptr) {
			QPointF sP = maybeBox->scenePos();
			maybeBox->setParentItem(this);
			maybeBox->setPos(sceneTransform().inverted().map(sP));
		}
	}
}

QVariant Zone::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
	// Snap zones to grid
	if (change == QGraphicsItem::ItemPositionChange && scene()) {
		// Get coordinate of the target new position
		QPointF targetPos = value.toPointF();

		// Get the scene in order to get the grid size
		DiagramScene *theScene = dynamic_cast<DiagramScene *>(scene());
		if (theScene == NULL) {
			informUserAndCrash("Could not cast the scene into a DiagramScene!");
		}
		int gridSize = theScene->gridSize();

		// Snap the new position's (x, y) coordinates to the grid
		qreal newX = round(targetPos.x() / gridSize) * gridSize;
		qreal newY = round(targetPos.y() / gridSize) * gridSize;

		// Create the Point representing the new, snapped position
		QPointF newPos(newX, newY);

		// Set the script to which this item's scene is associated as modified
		theScene->script()->setStatusModified(true);

		return newPos;
	}
	return QGraphicsRectItem::itemChange(change, value);
}

QColor Zone::color() const
{
	return m_color;
}

void Zone::setColor(const QColor &color)
{
	m_color = color;
}

QString Zone::title() const
{
	return m_title;
}

void Zone::setTitle(const QString &title)
{
	m_title = title;
}
