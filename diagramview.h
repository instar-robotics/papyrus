#ifndef DIAGRAMVIEW_H
#define DIAGRAMVIEW_H

#include <QGraphicsView>

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
