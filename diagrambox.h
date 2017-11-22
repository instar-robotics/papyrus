#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "arrow.h"

#include <QGraphicsItem>
#include <QGraphicsRectItem>

class DiagramBox : public QObject, public QGraphicsRectItem
{
    Q_OBJECT

public:
    explicit DiagramBox(QGraphicsItem *parent = 0);

    bool setStartLine(Arrow *line);
    bool setEndLine(Arrow *line);

    //void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

signals:
    //void positionChanged(bool isStartPoint);

private:
    Arrow *startLine;
    Arrow *endLine;

public slots:
};

#endif // DIAGRAMBOX_H
