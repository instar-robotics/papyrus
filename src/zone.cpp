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

Zone::Zone(qreal x, qreal y, qreal w, qreal h, QGraphicsObject *parent)
    : QGraphicsObject(parent),
      m_width(w),
      m_height(h),
      m_color(qRgba(51, 153, 255, 10)),
      m_resizeType(NO_RESIZE)
{
	m_color.setAlpha(80);

	setX(x);
	setY(y);

	setFlags(QGraphicsItem::ItemIsSelectable
	         | QGraphicsItem::ItemIsMovable
	         | QGraphicsItem::ItemSendsScenePositionChanges);
	setAcceptHoverEvents(true);

	setZValue(COMMENTS_Z_VALUE);
}

Zone::~Zone()
{

}

Zone::Zone(QGraphicsObject *parent) : Zone(0,0,0,0,parent)
{

}

QRectF Zone::boundingRect() const
{
	return QRectF(0, 0, m_width, m_height);
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

QVariant Zone::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
	if (change == QGraphicsItem::ItemPositionChange && scene()) {
		// Get coordinate of the target new position
		QPointF targetPos = value.toPointF();

		// Get the scene in order to get the grid size
		DiagramScene *theScene = dynamic_cast<DiagramScene *>(scene());
		if (theScene == nullptr) {
			informUserAndCrash("Could not cast the scene into a DiagramScene!");
		}
		int gridSize = theScene->gridSize();

		// Snap the new position's (x, y) coordinates to the grid
		qreal newX = round(targetPos.x() / gridSize) * gridSize;
		qreal newY = round(targetPos.y() / gridSize) * gridSize;

		// Create the Point representing the new, snapped position
		QPointF newPos(newX, newY);

		// Make the DiagramBoxe es children update their links when moving
		foreach (QGraphicsItem *child, childItems()) {
			DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(child);
			if (maybeBox != nullptr) {
				// This is a DIRTY trick: the itemChange() position was not called for the children
				// so I'm using this moveBy() to make it so that itemChange() is called.
				// This is  dirty trick but I could not find a way to propagate the itemChange() event
				// to the child items
				maybeBox->moveBy(0.1, 0);
				maybeBox->moveBy(-0.1, 0);
			}
		}

		return newPos;
	}
	return QGraphicsItem::itemChange(change, value);
}

void Zone::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	// Update the boxes included in this zone
	updateGroup();

	QPointF p(event->pos());

	// Flag resizing
	qreal margin = 10;
	if (p.x() <= margin) {
		m_resizeType = RESIZE_LEFT;
	} else if (p.x() >= m_width - margin) {
		m_resizeType = RESIZE_RIGHT;
	} else if (p.y() <= margin) {
		m_resizeType = RESIZE_TOP;
	} else if(p.y() > m_height - margin) {
		m_resizeType = RESIZE_BOTTOM ;
	} else {
		m_resizeType = NO_RESIZE;
	}

	QGraphicsObject::mousePressEvent(event);
}

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

	switch (m_resizeType) {
		// Resizing left can be done just by adjusting bottom right point
		case RESIZE_RIGHT:
//			m_width = event->pos().x();
			m_width = newX;
			update();
			theScene->update(); // COSTLY: TODO: find a better way
		break;

		case RESIZE_BOTTOM:
//			m_height = event->pos().y();
			m_height = newY;
			update();
			theScene->update(); // COSTLY: TODO: find a better way
		break;

		case RESIZE_TOP:
			sPos = event->scenePos();
			p = scenePos();
//			dy = p.y() - sPos.y();
			dy = round((p.y() - sPos.y()) / gridSize) * gridSize;
			p.setY(sPos.y());
			setPos(p);
			m_height += dy;
			update();
		break;

		case RESIZE_LEFT:
			sPos = event->scenePos();
			p = scenePos();
//			dx = p.x() - sPos.x();
			dx = round((p.x() - sPos.x()) / gridSize) * gridSize;
			p.setX(sPos.x());
			setPos(p);
			m_width += dx;

			// Move all children by the dx (because otherwise they move WITH the zone)
			foreach (QGraphicsItem *child, childItems()) {
				child->moveBy(dx, 0);
			}

			update();
		break;

		default:
			QGraphicsItem::mouseMoveEvent(event);
	}
}

void Zone::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	m_resizeType = NO_RESIZE;

	updateGroup(true);

	updateLinks();

	QGraphicsItem::mouseReleaseEvent(event);
}

void Zone::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
	QPointF p = event->pos();
	qreal margin = 10;

	if (p.x() <= margin) {
		setCursor(Qt::SizeHorCursor);
	} else if (p.x() >= m_width - margin) {
		setCursor(Qt::SizeHorCursor);
	} else if (p.y() <= margin) {
		setCursor(Qt::SizeVerCursor);
	} else if(p.y() > m_height - margin) {
		setCursor(Qt::SizeVerCursor);
	} else {
		setCursor(Qt::ArrowCursor);
	}

	QGraphicsItem::hoverMoveEvent(event);
}

/**
 * @brief Zone::updateGroup makes sure that all @DiagramBox es that are colliding with this zone
 * are made children of it.
 */
void Zone::updateGroup(bool startFromScratch)
{
	// If startFromScratch is true, first empty all children before starting over
	if (startFromScratch) {
		foreach (QGraphicsItem *child, childItems()) {

			QPointF savedPos = child->scenePos();
			child->setParentItem(nullptr);
			child->setPos(savedPos);
		}
	}

	QList<QGraphicsItem *> colliding = collidingItems();

	foreach (QGraphicsItem *item, colliding) {
		// Filter by DiagramBox which are not already children
		DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(item);
		if (maybeBox != nullptr && maybeBox->parentItem() != this) {
			// For some reason, calling setParentItem() moves the object weirdly so here we save its
			// current scene position mapped to the zone's coordinates, and immediately restore it
			QPointF savedPos = mapFromScene(maybeBox->scenePos());
			maybeBox->setParentItem(this);
			maybeBox->setPos(savedPos);
		}
	}
}

/**
 * @brief Zone::updateLinks updates the displaying of the links for all boxes inside this zone
 */
void Zone::updateLinks()
{
	foreach (QGraphicsItem *child, childItems()) {
		DiagramBox *maybeBox= dynamic_cast<DiagramBox *>(child);
		if (maybeBox != nullptr) {
			maybeBox->outputSlot()->updateLinks();
			foreach (InputSlot *iSlot, maybeBox->inputSlots()) {
				iSlot->updateLinks();
			}
		}
	}
}

qreal Zone::width() const
{
	return m_width;
}

void Zone::setWidth(const qreal &width)
{
	m_width = width;
}

qreal Zone::height() const
{
	return m_height;
}

void Zone::setHeight(const qreal &height)
{
	m_height = height;
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
