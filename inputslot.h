#ifndef INPUTSLOT_H
#define INPUTSLOT_H

#include "slot.h"
#include "arrow.h"

#include <QString>
#include <set>

// Forward declaration here because "arrow.h" includes "inputslot.h"
class Arrow;

class InputSlot : public Slot
{
    Q_OBJECT
public:
    explicit InputSlot();
    explicit InputSlot(QString &name);

    bool multiple() const;
    void setMultiple(bool allowMultiple);

    std::set<Arrow *> inputs() const;

    void addInput(Arrow *input);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;

    void updateArrows();

    InputType inputType() const;
    void setInputType(const InputType &inputType);

    bool canLink() const;
    void setCanLink(bool canLink);

private:
    bool m_multiple;
    std::set<Arrow *> m_inputs; // The set of arrows connected to this slot
    InputType m_inputType;      // Indicate type and connectivity of this input
    bool m_canLink; // Indicate if this input can be linked to the current output when creating a Link
signals:
    void slotFull(); // Fired when trying to add a second input to a slot that doesn't allow multiple
};

#endif // INPUTSLOT_H
