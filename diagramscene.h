#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include <QGraphicsScene>


class DiagramScene : public QGraphicsScene
{
    Q_OBJECT

public:
    //explicit DiagramScene(const QString &name, QObject *parent = 0);
    explicit DiagramScene(QObject *parent = 0);
    //~DiagramScene();

signals:
    //void zoomIn();
    //void zoomOut();

public slots:

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);
    //void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent);
    //void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent);

private:
    //QString *sceneName;
    //bool leftBtnIsDown;
    //bool middleBtnIsDown;
    //QGraphicsLineItem *line;

};

#endif // DIAGRAMSCENE_H
