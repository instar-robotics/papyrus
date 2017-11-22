#ifndef ARROW_H
#define ARROW_H

#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QPointF>

class Arrow : public QObject, public QGraphicsLineItem
{
    Q_OBJECT

public:
    explicit Arrow(QGraphicsItem *parent = 0);
    Arrow(const QLineF &line, QGraphicsItem *parent = 0);

    void updatePosition(QPointF newPoint, bool isStartPoint);

signals:
    void deleted();

public slots:
    void boxDeleted();
};

#endif // ARROW_H
