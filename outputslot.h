#ifndef OUTPUTSLOT_H
#define OUTPUTSLOT_H

#include "slot.h"
#include "arrow.h"

#include <QString>
#include <set>

/**
 * @brief The specialized version of @Slot used for ouput.
 * Outputs slots react to the mouse by growing sighly when the mouse comes nears them, in order
 * to facilitate the creation of a link.
 */

// Forward declaration here because "arrow.h" includes "outputslot.h"
class Arrow;

class OutputSlot : public Slot
{
    Q_OBJECT
public:
    explicit OutputSlot();
    explicit OutputSlot(QString &name);

    std::set<Arrow *> outputs() const;

    void addOutput(Arrow *output);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;

    void mousePressEvent(QGraphicsSceneMouseEvent *evt);

    bool isDrawingLine() const;
    void setIsDrawingLine(bool isDrawingLine);

private:
    std::set<Arrow *> m_outputs; // The set of arrows which leaves this slot
    bool m_isDrawingLine;        // Indicate if we are drawing an outgoing link
};

#endif // OUTPUTSLOT_H
