#ifndef SLOT_H
#define SLOT_H

#include <QObject>
#include <QString>
#include <QGraphicsItem>

/**
 * @brief The Slot class defines an argument slot, i.e. an item will either receive
 * a connection from another box's output slot, or from which a connection leaves to
 * reach another box' input slot.
 * This class is meant to be subclassed (see @InputSlot and @OutputSlot).
 */

class Slot : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit Slot(QGraphicsItem *parent = 0);
    explicit Slot(QString &name, QGraphicsItem *parent = 0);
    ~Slot();

    QString name() const;
    void setName(const QString &name);

    virtual QRectF boundingRect() const = 0;
private:
    QString m_name;  // The name of this slot
};

#endif // SLOT_H
