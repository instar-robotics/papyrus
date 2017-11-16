#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include <QGraphicsScene>


class DiagramScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit DiagramScene(const QString &name, QObject *parent = 0);
    ~DiagramScene();

signals:
    void zoomIn();
    void zoomOut();

public slots:

signals:
    void itemSelected(QGraphicsItem *item);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent);
    void wheelEvent(QGraphicsSceneWheelEvent *wheelEvent);

private:
    QString *sceneName;
    bool leftBtnIsDown;
    QGraphicsLineItem *line;

};

#endif // DIAGRAMSCENE_H
