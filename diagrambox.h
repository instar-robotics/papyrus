#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include <QGraphicsItem>
#include <QGraphicsRectItem>

class DiagramBox : public QGraphicsRectItem
{
public:
    explicit DiagramBox(QGraphicsItem *parent = 0);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
signals:

public slots:
};

#endif // DIAGRAMBOX_H
