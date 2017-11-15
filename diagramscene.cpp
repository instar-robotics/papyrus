#include "diagramscene.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>

DiagramScene::DiagramScene(QObject *parent) : QGraphicsScene(parent)
{
    line = 0;
    myLineColor = Qt::black;
    myItemColor = Qt::blue;
}

void DiagramScene::setLineColor(const QColor &color)
{
    myLineColor = color;
}

void DiagramScene::setItemColor(const QColor &color)
{
    myItemColor = color;
}

void DiagramScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;

    QGraphicsRectItem *item;
    // Only insert mode for now
    item = new QGraphicsRectItem(0, 0, 150, 100);
    addItem(item);
    item->setPos(mouseEvent->scenePos());
}

void DiagramScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    // No nothing for now
    QGraphicsScene::mouseMoveEvent(mouseEvent);
}

void DiagramScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    // Do nothing for now
    QGraphicsScene::mouseReleaseEvent(mouseEvent);
}
