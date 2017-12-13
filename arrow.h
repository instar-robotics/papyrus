#ifndef ARROW_H
#define ARROW_H

#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QPointF>

// Forward declaration here because "diagrambox.h" includes "arrow.h"
class DiagramBox;

class Arrow : public QObject, public QGraphicsLineItem
{
    Q_OBJECT

public:
    static int nb;

    static int getType();

    explicit Arrow(QGraphicsItem *parent = 0);
    Arrow(const QLineF &line, QGraphicsItem *parent = 0);

    void updatePosition(QPointF newPoint, bool isStartPoint);

    DiagramBox *from() const {return from_;}
    DiagramBox *to() const {return to_;}

    void setFrom(DiagramBox *box);
    void setTo(DiagramBox *box);

    QPainterPath shape() const override;

    int type() const;

    int no; // TEMP

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0) override;

private:
    DiagramBox *from_; // Box from which this Arrow starts
    DiagramBox *to_;   // Box to which this Arrow ends
    QPolygonF arrowHead;
};

#endif // ARROW_H
