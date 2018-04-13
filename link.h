#ifndef LINK_H
#define LINK_H

#include <QGraphicsItem>
#include <QUuid>
#include <QPainterPath>
#include <QGraphicsLineItem>

/**
 * @brief The Link class represents a link between neural functions. More specifically, a Link is
 * between thee @OutputSlot of a @DiagramBox and the @InputSlot of (another or the same) @DiagramBox
 */

class InputSlot;
class OutputSlot;

/**
 * @brief The LinkOperation enum is used to set the type of operation that is performed between
 * the input and the weights of an input.
 * BE CAREFUL: the order of the definition here must match the order in which they are presented in
 * the QComboBox in the @PropertiesPanel (because of setCurrentIndex())
 */
enum LinkOperation {
    OP_PRODUCT,     // input * weight
    OP_ADDITION,    // input + weigth
    OP_SUBTRACTION, // input - weight
    OP_DIVISION    // input / weight
};

Q_DECLARE_METATYPE(LinkOperation) // This allows convertion from/to QVariant

class Link : public QObject, public QGraphicsItem
{
    Q_OBJECT

public:
    explicit Link(OutputSlot *f, InputSlot *t, QGraphicsItem *parent = 0);
    ~Link();

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
    QPainterPath shape() const;

    void addLinesToScene();

    void updateLines();

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

    LinkOperation operation() const;
    void setOperation(const LinkOperation &operation);

private:
    bool checkIfSecondary();

    QUuid m_uuid;           // Unique identifier
    OutputSlot *m_from;     // The OutputSlot this link goes from
    InputSlot *m_to;        // The InputSlot this link goes to
    bool m_secondary;       // Tells whether a link is a secondary link, i.e. self-looping

    QGraphicsLineItem m_line;          // Main line that represents the link
    QGraphicsLineItem m_rightSegment;  // Right segment (for secondary links)
    QGraphicsLineItem m_leftSegment;   // Left segment (for secondary links)

    qreal m_weight;            // The weight associated to this link
    LinkOperation m_operation; // The operator used between the inputs and the weights
};

#endif // LINK_H
