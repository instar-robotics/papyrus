#ifndef INPUTSLOT_H
#define INPUTSLOT_H

#include "slot.h"
//#include "arrow.h"

#include <QString>
#include <set>

// Forward declaration here because "arrow.h" includes "inputslot.h"
// Forward declaration here because we need the Arrow type, but "arrow.h" includes "inputsslot.h"
class Arrow;

class InputSlot : public Slot
{
    Q_OBJECT
public:
    explicit InputSlot();
    explicit InputSlot(QString &name);

    bool allowMultiple() const;
    void setAllowMultiple(bool allowMultiple);

    std::set<Arrow *> inputs() const;

    void addInput(Arrow *input);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;
private:
    bool m_allowMultiple;
    std::set<Arrow *> m_inputs; // The set of arrows connected to this slot
signals:
    void slotFull(); // Fired when trying to add a second input to a slot that doesn't allow multiple
};

#endif // INPUTSLOT_H
