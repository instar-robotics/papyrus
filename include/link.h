#ifndef LINK_H
#define LINK_H
#include "script.h"

#include <QGraphicsItem>
#include <QUuid>
#include <QPainterPath>
#include <QGraphicsLineItem>

/**
 * @brief The Link class represents a link between neural functions (more precisely between
 * @DiagramBox es) Even more precisely, a Link is between a @DiagramBox 's @OutputSlot and
 * another (or same) @DiagramBox 's @InputSlot.
 * Depending on its type, it can have a weight.
 */

class InputSlot;
class OutputSlot;

class Link : public QObject, public QGraphicsItem
{
    Q_OBJECT

public:
    explicit Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent = 0);

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
    QPainterPath shape() const;

    void addLinesToScene();

    void updateLines();

    bool checkIfInvalid();

    QUuid uuid() const;
    void setUuid(const QUuid &uuid);

    OutputSlot *from() const;
    void setFrom(OutputSlot *from);

    InputSlot *to() const;
    void setTo(InputSlot *to);

    bool secondary() const;
    void setSecondary(bool secondary);

    qreal weight() const;
    void setWeight(const qreal &weight);

    bool isInvalid() const;
    void setIsInvalid(bool isInvalid);

    bool selfLoop() const;

private:
    bool checkIfSelfLoop();

    QUuid m_uuid;           // Unique identifier
    OutputSlot *m_from;     // The OutputSlot this link goes from
    InputSlot *m_to;        // The InputSlot this link goes to
    bool m_secondary;       // Tells whether a link is a secondary link
    bool m_selfLoop;        // Tells whether a link loop back to the same function

    QGraphicsLineItem m_line;          // Main line that represents the link
    QGraphicsLineItem m_rightSegment;  // Right segment (for secondary links)
    QGraphicsLineItem m_leftSegment;   // Left segment (for secondary links)

    qreal m_weight;            // The weight associated to this link

    bool m_isInvalid; // Tells that this link is currently not valid (error in type, in sizes, etc.)
};

#endif // LINK_H