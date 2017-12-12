#ifndef DIAGRAMVIEW_H
#define DIAGRAMVIEW_H

#include <QGraphicsView>

class DiagramView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit DiagramView(QWidget *parent = 0);
    DiagramView(QGraphicsScene *scene, QWidget *parent = 0);
    QString foo;

signals:

protected:
    void wheelEvent(QWheelEvent *evt);
    void mousePressEvent(QMouseEvent *evt);

//    void dragEnterEvent(QDragEnterEvent *evt);
//    void dragLeaveEvent(QDragLeaveEvent *evt);
//    void dragMoveEvent(QDragMoveEvent *evt);
//    void dropEvent(QDropEvent *evt);

public slots:
};

#endif // DIAGRAMVIEW_H
