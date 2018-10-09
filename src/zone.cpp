#include "zone.h"
#include "diagrambox.h"
#include "constants.h"

#include <QDebug>
#include <QPen>
#include <QBrush>
#include <QPainter>
#include <QPainterPath>
#include <QGraphicsScene>

Zone::Zone(qreal x, qreal y, qreal w, qreal h, QGraphicsObject *parent)
    : QGraphicsObject(parent),
      m_width(w),
      m_height(h),
      m_color(qRgba(51, 153, 255, 10))
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
//	delete m_group;
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

	QPen pen;

	pen.setColor(Qt::red);

	painter->setPen(pen);

	QPainterPath path;
	path.addRoundedRect(boundingRect(), 4, 4);
	painter->fillPath(path, m_color);
}

QVariant Zone::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
	// When it is moved, we need to move the functions inside it
	if (change == QGraphicsItem::ItemPositionChange && scene()) {

	}

	return QGraphicsItem::itemChange(change, value);
}

void Zone::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	updateGroup();

	QGraphicsObject::mousePressEvent(event);
}

/**
 * @brief Zone::updateGroup makes sure that all @DiagramBox es that are colliding with this zone
 * are made children of it.
 */
void Zone::updateGroup()
{

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
