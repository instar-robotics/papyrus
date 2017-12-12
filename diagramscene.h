#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include "script.h"

#include <QGraphicsScene>
#include <diagrambox.h>
#include <arrow.h>
#include <QUuid>

// Deferred declaration because of recursive include
class Script;

class DiagramScene : public QGraphicsScene
{
    Q_OBJECT

    void updateSceneRect();

public:
    explicit DiagramScene(QObject *parent = 0);
    ~DiagramScene();

    void addBox(const QPointF &position, const QString &name, const QIcon &icon, QUuid uuid = 0);

    bool shouldDrawGrid() const;
    void setShouldDrawGrid(bool shouldDrawGrid);

    int gridSize() const;

    Script *script() const;
    void setScript(Script *script);

public slots:
    void toggleDisplayGrid(bool shouldDraw);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *evt);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
    void dragEnterEvent(QGraphicsSceneDragDropEvent *evt);
    void dragLeaveEvent(QGraphicsSceneDragDropEvent *evt);
    void dragMoveEvent(QGraphicsSceneDragDropEvent *evt);
    void dropEvent(QGraphicsSceneDragDropEvent *evt);
    void keyPressEvent(QKeyEvent *evt);
    void removeItem(QGraphicsItem *item);
    void removeItem(Arrow *arrow);
    void removeItem(DiagramBox *box);

    void drawBackground(QPainter *painter, const QRectF &rect);

private:
    //QString *sceneName;
    //bool leftBtnIsDown;
    bool middleBtnIsDown;
    bool m_shouldDrawGrid;   // Whether to draw the grid or not
    int m_gridSize;          // Size (in px) of the grid
    QGraphicsLineItem *line; // The current line being drawn while clicking
    DiagramBox *box;         // The current box from which the current line originates
    Script *m_script;        // The script to which this scene is associated

signals:
    void displayStatusMessage(const QString &text);
};

#endif // DIAGRAMSCENE_H
