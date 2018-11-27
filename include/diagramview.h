#ifndef DIAGRAMVIEW_H
#define DIAGRAMVIEW_H

#include <QGraphicsView>

/**
 * @brief The DiagramView class is used to render a @DiagramScene (the @DiagramScene holds
 * the data: the items added, their positions, etc.) and the DiagramView displays it. This
 * separation by Qt allows several views to be attached to a single scene (we will use
 * something like this when we implement the minimap).
 */
class DiagramView : public QGraphicsView
{
	Q_OBJECT
public:
	explicit DiagramView(QWidget *parent = 0);
	DiagramView(QGraphicsScene *scene, QWidget *parent = 0);

signals:

protected:
	void wheelEvent(QWheelEvent *evt);

public slots:
};

#endif // DIAGRAMVIEW_H
