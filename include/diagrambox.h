#ifndef DIAGRAMBOX_H
#define DIAGRAMBOX_H

#include "types.h"
#include "slot.h"
#include "outputslot.h"
#include "inputslot.h"

#include <set>

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QGraphicsSvgItem>

/**
 * @brief The DiagramBox class is the main class that represents a "box" or "neural function".
 * This has a name, an icon, a type (scalar or matrix), etc.
 * It can have @InputSlot s and @OutputSlot s attached.
 */

class DiagramBox : public QObject, public QGraphicsItem
{
    Q_OBJECT

public:
    static int getType();


    // TODO: implement a copy constructor that should change the uuid and remove the connected links
    explicit DiagramBox(const QString &name,
                        const QIcon &icon,
                        OutputSlot *outputSlot,
                        std::set<InputSlot *> inputSlots,
                        const QUuid &uuid = 0,
                        QGraphicsItem *parent = 0);
    ~DiagramBox();

    QRectF boundingRect() const override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

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

    OutputType outputType() const;
    void setOutputType(const OutputType &outputType);

    int rows() const;
    void setRows(int rows);

    int cols() const;
    void setCols(int cols);

    bool saveActivity() const;
    void setSaveActivity(bool saveActivity);

    qreal bWidth() const;

    qreal bHeight() const;

    qreal tHeight() const;

    QGraphicsSvgItem *sizeIcon() const;
    void setSizeIcon(QGraphicsSvgItem *sizeIcon);

    bool publish() const;
    void setPublish(bool publish);

    QString topic() const;
    void setTopic(const QString &topic);

    QString iconFilepath() const;
    void setIconFilepath(const QString &value);

    QString libname() const;
    void setLibname(const QString &libname);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private:
    QUuid m_uuid;      // Unique ID of the function's box (to identify links for instance)
    QString m_name;    // Name of the function
    QString m_libname; // Name of the library this function belongs to (for kheops's linking)
    QIcon m_icon;      // Icon representing the function

    qreal m_bWidth;  // Overall width of the function's box
    qreal m_bHeight; // Overall height of the function's box
    qreal m_tHeight; // Height of the space in which th function's name is written

    QString m_descriptionFile; // Path to its XML description file (to get the icon when saving)

    OutputSlot *m_outputSlot;  // The output slot for this function's box
    std::set<InputSlot *> m_inputSlots; // The set of input slots for this function's box

    int m_rows;              // Number of rows in the output (if matrix)
    int m_cols;              // Number of columns in the output (if matrix)

    /*
     * Whether or not this function saves its activity in memory. It defaults to false. Saving the
     * activity is not free and should only be done for activities that you need to recover if
     * the script crashes and the robot cannot rebuild from its environment.
     * For instance the path integration, or a counter, or smth.
     * This should normally only be scalar values or small matrices
     */
    bool m_saveActivity;
    bool m_publish;           // whether to publish this function's output on ROS
    QString m_topic;          // name of the topic in which to publish the output

    QString m_iconFilepath;         // Filepath of the icon used (needed to create the svg)
    QGraphicsSvgItem *m_sizeIcon; // Contains the svg that hints the box's size

signals:
    void boxSelected(DiagramBox *); // Fired when the box is clicked on (used to signal PropertiesPanel)
};

#endif // DIAGRAMBOX_H
