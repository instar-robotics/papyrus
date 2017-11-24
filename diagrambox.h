#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "arrow.h"

#include <set>

#include <QGraphicsItem>
#include <QGraphicsRectItem>

class DiagramBox : public QObject, public QGraphicsRectItem
{
    Q_OBJECT

public:
    static int nb;

    explicit DiagramBox(QGraphicsItem *parent = 0);

    std::set<Arrow *> startLines() const {return startLines_;}
    std::set<Arrow *> endLines() const {return endLines_;}

    void addStartLine(Arrow *line);
    void addEndLine(Arrow *line);
    void removeStartLine(Arrow *line);
    void removeEndLine(Arrow *line);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    int no; // TEMP

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private:
    //std::vector<Arrow *> startLines_; // The list of Arrows originating from this Box
    //std::vector<Arrow *> endLines_;   // The list of Arrows pointing to this Box
    std::set<Arrow *> startLines_; // The list of Arrows originating from this Box
    std::set<Arrow *> endLines_;   // The list of Arrows pointing to this Box
};

#endif // DIAGRAMBOX_H
