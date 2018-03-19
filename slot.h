#ifndef SLOT_H
#define SLOT_H

#include <QObject>
#include <QString>
#include <QGraphicsItem>

//#include <diagrambox.h>
class DiagramBox;

/**
 * @brief The Slot class defines an argument slot, i.e. an item will either receive
 * a connection from another box's output slot, or from which a connection leaves to
 * reach another box' input slot.
 * This class is meant to be subclassed (see @InputSlot and @OutputSlot).
 */

enum InputType {
    SCALAR_SCALAR,
    SIMPLE_MATRIX,
    SCALAR_MATRIX,
    MATRIX_MATRIX
};

enum OutputType {
    SCALAR,
    MATRIX
};

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

    qreal dist() const;
    void setDist(const qreal &dist);

    DiagramBox *box() const;
    void setBox(DiagramBox *box);

protected:
    QString m_name;    // The name of this slot
    qreal m_dist;      // Distance to the mouse (used to highlight the slot when mouse approach)
    DiagramBox *m_box; // The DiagramBox that is associated with this Slot
};

#endif // SLOT_H
