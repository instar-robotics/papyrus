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
      m_color(qRgba(51, 153, 255, 10)),
      m_group(new QGraphicsItemGroup(this))
{
	m_color.setAlpha(80);
	setX(x);
	setY(y);

	setFlags(QGraphicsItem::ItemIsSelectable
	         | QGraphicsItem::ItemIsMovable
	         | QGraphicsItem::ItemSendsScenePositionChanges);
	setAcceptHoverEvents(true);

	setZValue(COMMENTS_Z_VALUE);

	m_group->setZValue(COMMENTS_Z_VALUE);
	m_group->setFlag(QGraphicsItem::ItemIsSelectable, true);
	m_group->setAcceptedMouseButtons(0);
}

Zone::~Zone()
{
	delete m_group;
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

void Zone::updateGroup()
{
	// First, add the group to the scene if it is not already added
//	if (m_group->scene() == nullptr && scene() != nullptr) {
//		scene()->addItem(m_group);
//	}

	QList<QGraphicsItem *> colliding = collidingItems();

	foreach (QGraphicsItem *item, colliding) {
		// Add the DiagramBox only in the group
		DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(item);
		if (maybeBox != nullptr) {
			m_group->addToGroup(item);
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

QGraphicsItemGroup *Zone::group() const
{
	return m_group;
}

void Zone::setGroup(QGraphicsItemGroup *group)
{
	m_group = group;
}
