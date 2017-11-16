#include "diagramscene.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <iostream>

DiagramScene::DiagramScene(const QString &name, QObject *parent) : QGraphicsScene(parent)
{
    sceneName = new QString(name);
    setSceneRect(0, 0, 5000, 5000);
    leftBtnIsDown = false;
    line = 0;
}

DiagramScene::~DiagramScene()
{
    delete sceneName;
    std::cout << "Destructed" << std::endl;
}

void DiagramScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;

    leftBtnIsDown = true;

    // Check if we have clicked on something
    QGraphicsItem *maybeItem = itemAt(mouseEvent->scenePos(), QTransform());
    if (!maybeItem) {
        // Only insert mode for now
        QGraphicsRectItem *item = new QGraphicsRectItem(0, 0, 150, 100);
        addItem(item);
        QPointF p = item->boundingRect().bottomRight();
        item->setPos(mouseEvent->scenePos() - p / 2);
    } else {
        QPointF start = maybeItem->scenePos();
        QRectF box = maybeItem->boundingRect();
        qreal w = box.width();
        qreal h = box.height();
        QPointF end = QPointF(start.x() + w, start.y() + h);

        line = new QGraphicsLineItem(QLineF((start + end) / 2,
                                     mouseEvent->scenePos()));
        addItem(line);
    }

    QGraphicsScene::mousePressEvent(mouseEvent);
}

void DiagramScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (leftBtnIsDown && line != 0) {
        QLineF newLine(line->line().p1(), mouseEvent->scenePos());
        line->setLine(newLine);
    } else {
        // No nothing for now
        QGraphicsScene::mouseMoveEvent(mouseEvent);
    }
}

void DiagramScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
    if (mouseEvent->button() != Qt::LeftButton)
        return;

    leftBtnIsDown = false;

    if (line != 0) {
        removeItem(line);

        // Check if we have released on something
        QGraphicsItem *maybeItem = itemAt(mouseEvent->scenePos(), QTransform());

        if (maybeItem) {
            QPointF start = maybeItem->scenePos();
            QRectF box = maybeItem->boundingRect();
            qreal w = box.width();
            qreal h = box.height();
            QPointF end = QPointF(start.x() + w, start.y() + h);
            QGraphicsLineItem *finalLine = new QGraphicsLineItem(QLineF(line->line().p1(), (start + end) / 2));
            finalLine->setPen(QPen(Qt::blue, 2));
            addItem(finalLine);
        }

        delete line;
        line = 0;
    }
}

void DiagramScene::wheelEvent(QGraphicsSceneWheelEvent *wheelEvent)
{
    if (wheelEvent->delta() > 0)
        emit zoomIn();
    else
        emit zoomOut();

    return;
}
