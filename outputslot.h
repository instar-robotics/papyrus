#ifndef OUTPUTSLOT_H
#define OUTPUTSLOT_H

#include "slot.h"
#include "arrow.h"

#include <QString>
#include <set>

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

private:
    std::set<Arrow *> m_outputs; // The set of arrows which leaves this slot
};

#endif // OUTPUTSLOT_H
