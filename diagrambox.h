#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "arrow.h"
#include "slot.h"
#include "outputslot.h"
#include "inputslot.h"

#include <set>

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>

class DiagramBox : public QObject, public QGraphicsItem
{
    Q_OBJECT

public:
    static int getType();
    // TODO: implement a copy constructor that should change the uuid and remove the connected arrows
    explicit DiagramBox(const QString &name,
                        const QIcon &icon,
                        OutputSlot *outputSlot,
                        std::set<InputSlot *> inputSlots,
                        QUuid uuid = 0,
                        QGraphicsItem *parent = 0);
    ~DiagramBox();

    QRectF boundingRect() const override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    std::set<Arrow *> startLines() const {return startLines_;}
    std::set<Arrow *> endLines() const {return endLines_;}

    void addStartLine(Arrow *line);
    void addEndLine(Arrow *line);
    void removeStartLine(Arrow *line);
    void removeEndLine(Arrow *line);

    QString name() const;
    void setName(const QString &name);

    QUuid uuid() const;

    int type();

    QString descriptionFile() const;
    void setDescriptionFile(const QString &descriptionPath);

    QIcon icon() const;
    void setIcon(const QIcon &icon);

    OutputSlot *outputSlot() const;
    void setOutputSlot(OutputSlot *outputSlot);

    std::set<InputSlot *> inputSlots() const;
    void setInputSlots(const std::set<InputSlot *> &inputSlots);

    void mousePressEvent(QGraphicsSceneMouseEvent *evt);

    OutputType outputType() const;
    void setOutputType(const OutputType &outputType);

    int rows() const;
    void setRows(int rows);

    int cols() const;
    void setCols(int cols);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private:
    QUuid m_uuid;   // Unique ID of the function's box (to identify links for instance)
    QString m_name; // Name of the function
    QIcon m_icon;   // Icon representing the function

    std::set<Arrow *> startLines_; // The list of Arrows originating from this Box
    std::set<Arrow *> endLines_;   // The list of Arrows pointing to this Box

    qreal bWidth;  // Overall width of the function's box
    qreal bHeight; // Overall height of the function's box
    qreal tHeight; // Height of the space in which th function's name is written

    QString m_descriptionFile; // Path to its XML description file (to get the icon when saving)

    OutputSlot *m_outputSlot;  // The output slot for this function's box
    std::set<InputSlot *> m_inputSlots; // The set of input slots for this function's box

    int m_rows;              // Number of rows in the output (if matrix)
    int m_cols;              // Number of columns in the output (if matrix)

signals:
    void boxSelected(DiagramBox *); // Fired when the box is clicked on (used ot signal PropertiesPanel)
};

#endif // DIAGRAMBOX_H
