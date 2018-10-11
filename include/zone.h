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
