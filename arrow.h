#ifndef ARROW_H
#define ARROW_H

#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QPointF>

#include <outputslot.h>
#include <inputslot.h>

// Forward declaration here because "diagrambox.h" includes "arrow.h"
class DiagramBox;

// Forward declaration here because "inputslot.h" includes "arrow.h"
class InputSlot;

// Forward declaration here because "outputslot.h" includes "arrow.h"
class OutputSlot;

class Arrow : public QObject, public QGraphicsLineItem
{
    Q_OBJECT

public:
    static int nb;

    static int getType();

    explicit Arrow(QGraphicsItem *parent = 0);
    Arrow(const QLineF &line, QGraphicsItem *parent = 0);

    void updatePosition(QPointF newPoint, bool isStartPoint);

    OutputSlot *from() const {return m_from;}
    InputSlot *to() const {return m_to;}

    void setFrom(OutputSlot *box);
    void setTo(InputSlot *box);

    QPainterPath shape() const override;

    int type() const;

    int no; // TEMP

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0) override;

private:
    OutputSlot *m_from; // Output slot from which this Arrow starts
    InputSlot *m_to;   // Input slot to which this Arrow ends
    QPolygonF arrowHead;
};

#endif // ARROW_H
