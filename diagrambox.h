#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "arrow.h"

#include <set>

#include <QGraphicsItem>
#include <QIcon>

class DiagramBox : public QGraphicsItem
{
public:
    static int nb;

    explicit DiagramBox(const QString &name, const QIcon &icon, QGraphicsItem *parent = 0);

    QRectF boundingRect() const override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    std::set<Arrow *> startLines() const {return startLines_;}
    std::set<Arrow *> endLines() const {return endLines_;}

    void addStartLine(Arrow *line);
    void addEndLine(Arrow *line);
    void removeStartLine(Arrow *line);
    void removeEndLine(Arrow *line);

    int no; // TEMP

    QString name() const;
    void setName(const QString &name);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private:
    QString m_name;
    QIcon m_icon;

    std::set<Arrow *> startLines_; // The list of Arrows originating from this Box
    std::set<Arrow *> endLines_;   // The list of Arrows pointing to this Box

    qreal bWidth;    // Overall width of the function's box
    qreal bHeight;   // Overall height of the function's box
    qreal tHeight;   // Height of the space in which th function's name is written
};

#endif // DIAGRAMBOX_H
