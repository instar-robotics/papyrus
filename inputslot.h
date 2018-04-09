#ifndef INPUTSLOT_H
#define INPUTSLOT_H

#include "slot.h"

#include <QString>
#include <set>
#include <QGraphicsSimpleTextItem>

// Forward declaration here because "arrow.h" includes "inputslot.h"
class Link;

class InputSlot : public Slot
{
    Q_OBJECT

public:
    explicit InputSlot();
    explicit InputSlot(QString &name);
    ~InputSlot();

    bool multiple() const;
    void setMultiple(bool allowMultiple);

    std::set<Link *> inputs() const;

    void addInput(Link *input);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;

    void updateLinks();

    InputType inputType() const;
    void setInputType(const InputType &inputType);

    bool canLink() const;
    void setCanLink(bool canLink);

    QGraphicsSimpleTextItem *label() const;
    void setLabel(QGraphicsSimpleTextItem *label);

private:
    bool m_multiple;
    std::set<Link *> m_inputs; // The set of arrows connected to this slot
    InputType m_inputType;      // Indicate type and connectivity of this input
    bool m_canLink; // Indicate if this input can be linked to the current output when creating a Link
    QGraphicsSimpleTextItem *m_label;  // A label that contains this input's name
signals:
    void slotFull(); // Fired when trying to add a second input to a slot that doesn't allow multiple
};

#endif // INPUTSLOT_H
