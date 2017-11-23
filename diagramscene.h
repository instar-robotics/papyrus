#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include <QGraphicsScene>
#include <diagrambox.h>
#include <arrow.h>


class DiagramScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit DiagramScene(QObject *parent = 0);
    ~DiagramScene();

public slots:

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *evt);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
    void keyPressEvent(QKeyEvent *evt);
    void removeItem(QGraphicsItem *item);
    void removeItem(Arrow *arrow);
    void removeItem(DiagramBox *box);

private:
    //QString *sceneName;
    //bool leftBtnIsDown;
    bool middleBtnIsDown;
    QGraphicsLineItem *line; // The current line being drawn while clicking
    DiagramBox *box;         // The current box from which the current line originates

};

#endif // DIAGRAMSCENE_H
