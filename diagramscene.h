#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include "script.h"
#include "outputslot.h"
#include "inputslot.h"

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

    DiagramBox* addBox(const QPointF &position,
                       const QString &name,
                       const QIcon &icon,
                       OutputSlot *outputSlot,
                       std::set<InputSlot *> inputSlots,
                       QUuid uuid = 0);

    bool shouldDrawGrid() const;
    void setShouldDrawGrid(bool shouldDrawGrid);

    int gridSize() const;

    Script *script() const;
    void setScript(Script *script);

    void createLink(OutputSlot *from, InputSlot *to);

    bool leftBtnDown() const;

    QGraphicsLineItem *line() const;

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
    bool m_leftBtnDown;
    bool middleBtnIsDown;
    bool m_shouldDrawGrid;   // Whether to draw the grid or not
    int m_gridSize;          // Size (in px) of the grid
    QGraphicsLineItem *m_line; // The current line being drawn while clicking
    OutputSlot *m_oSlot;     // The slot from which the line being drawn originates
    Script *m_script;        // The script to which this scene is associated

private slots:
    void onSelectionChanged();

signals:
    void displayStatusMessage(const QString &text);

};

#endif // DIAGRAMSCENE_H
