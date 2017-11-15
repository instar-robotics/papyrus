#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include <QGraphicsScene>


class DiagramScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit DiagramScene(QObject *parent = 0);
    QColor lineColor() const { return myLineColor; }
    QColor itemColor() const { return myItemColor; }
    void setLineColor(const QColor &color);
    void setItemColor(const QColor &color);

public slots:

signals:
    void itemSelected(QGraphicsItem *item);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) override;

private:
    QColor myLineColor;
    QColor myItemColor;
    QPointF startPoint;
    QGraphicsLineItem *line;

};

#endif // DIAGRAMSCENE_H
