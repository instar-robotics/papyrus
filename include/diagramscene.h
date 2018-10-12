#ifndef DIAGRAMSCENE_H
#define DIAGRAMSCENE_H

#include "script.h"
#include "outputslot.h"
#include "inputslot.h"
#include "diagrambox.h"
#include "zone.h"

#include <QGraphicsScene>
#include <QUuid>

// Forward declaration because of recursive include
class Script;
class PapyrusWindow;

/**
 * @brief The DiagramScene class represents the canvas on which neural scripts are drawn.
 * This is where @DiagramBox es are drag-n-dropped. There is one DiagramScene per script.
 */
class DiagramScene : public QGraphicsScene
{
	Q_OBJECT

public:
	explicit DiagramScene(QObject *parent = 0);
	~DiagramScene();

	void addBox(DiagramBox *newBox, const QPointF &position);

	bool checkForInvalidLinks();

	void updateSceneRect();

	bool shouldDrawGrid() const;
	void setShouldDrawGrid(bool shouldDrawGrid);

	int gridSize() const;

	Script *script() const;
	void setScript(Script *script);

	bool leftBtnDown() const;

	QGraphicsLineItem *line() const;

	PapyrusWindow *mainWindow() const;

	bool displayLabels() const;
	void setDisplayLabels(bool displayLabels);

	bool rightBtnDown() const;
	void setRightBtnDown(bool rightBtnDown);

	QGraphicsRectItem *rect() const;
	void setRect(QGraphicsRectItem *rect);

public slots:
	void toggleDisplayGrid(bool shouldDraw);
	void onOkBtnClicked(bool);
	void onCancelBtnClicked(bool);
	void onDisplayVisuClicked(bool);

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *evt);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *evt);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
	void dragEnterEvent(QGraphicsSceneDragDropEvent *evt);
	void dragLeaveEvent(QGraphicsSceneDragDropEvent *evt);
	void dragMoveEvent(QGraphicsSceneDragDropEvent *evt);
	void dropEvent(QGraphicsSceneDragDropEvent *evt);
	void keyPressEvent(QKeyEvent *evt);
	void deleteItem(Link *link);
	void deleteItem(DiagramBox *box);
	void deleteItem(Zone *zone);

	void removeItem(QGraphicsItem *item);

	void drawBackground(QPainter *painter, const QRectF &rect);

private:
	PapyrusWindow *m_mainWindow; // A pointer to the main Papyrus window widget
	bool m_leftBtnDown;
	bool middleBtnIsDown;
	bool m_rightBtnDown;
	bool m_shouldDrawGrid;   // Whether to draw the grid or not
	int m_gridSize;          // Size (in px) of the grid
	QGraphicsLineItem *m_line; // The current line being drawn while clicking
	QGraphicsRectItem *m_rect; // The current rectangular section being drawn while clicking
	OutputSlot *m_oSlot;     // The slot from which the line being drawn originates
	Script *m_script;        // The script to which this scene is associated
	bool m_displayLabels;    // Whether or not to display input slots's names
	bool m_prevDisplayLabels;// Remembers the value of 'displayLabel' when creating links (to restore afterward)

private slots:
	void onSelectionChanged();

signals:
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);

};

#endif // DIAGRAMSCENE_H
