#include "diagrambox.h"
#include "diagramscene.h"
#include "papyruswindow.h"

#include <iostream>

#include <QApplication>
#include <QPen>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <math.h>
#include <QDebug>

int DiagramBox::getType()
{
    return UserType + 1;
}

DiagramBox::DiagramBox(const QString &name,
                       const QIcon &icon,
                       OutputSlot *outputSlot,
                       std::set<InputSlot *> inputSlots,
                       QUuid uuid,
                       QGraphicsItem *parent) : QGraphicsItem(parent),
                                                m_uuid(uuid),
                                                m_name(name),
                                                m_icon(icon),
                                                bWidth(175),
                                                bHeight(70),
                                                tHeight(20),
                                                m_outputSlot(outputSlot),
                                                m_inputSlots(inputSlots),
                                                m_rows(0),
                                                m_cols(0),
                                                m_saveActivity(false)
{
    // Generate a UUID if there was not one while created
    if (m_uuid.isNull())
        m_uuid = QUuid::createUuid();

    setFlags(QGraphicsItem::ItemIsSelectable
           | QGraphicsItem::ItemIsMovable
           | QGraphicsItem::ItemSendsScenePositionChanges);
    setAcceptHoverEvents(true);

    // Make this the parent item of the output slot, so that it follow drags, etc.
    m_outputSlot->setParentItem(this);
    m_outputSlot->setBox(this);
    m_outputSlot->setAcceptHoverEvents(true);

    // Set the output's slot position, in its parent's referential (this item's)
    QPointF p = (boundingRect().bottomRight() + boundingRect().topRight()) / 2;
    p.rx() += 5; // Set a bit of margin to the right to prevent the diamon-shape to overlap
    m_outputSlot->setPos(p);

    // Make this the parent item of all input slots, so that they follow drags, etc.
    qreal s = 20;
    qreal offset = m_inputSlots.size() % 2 == 0 ? s / 2 : 0; // offset only if even nb of slots

    QPointF g = (boundingRect().bottomLeft() + boundingRect().topLeft()) / 2;
    g.rx() -= 5;

    g.ry() -= ((m_inputSlots.size() - 1) / 2) * s + offset;

    // Get the PropertiesPanel and connect its display slot to this box's signal
    PapyrusWindow *mainWindow = NULL;

    foreach (QWidget *w, qApp->topLevelWidgets()) {
        if (PapyrusWindow *mW = qobject_cast<PapyrusWindow *>(w)) {
            mainWindow = mW;
            break;
        }
    }

    if (mainWindow) {
        PropertiesPanel *propertiesPanel = mainWindow->propertiesPanel();

        if (propertiesPanel == NULL)
            qFatal("PropertiesPanel doesn't exist!");

        connect(this, SIGNAL(boxSelected(DiagramBox *)), propertiesPanel, SLOT(displayBoxProperties(DiagramBox *)));
    }

    foreach (InputSlot *inputSlot, m_inputSlots) {
        inputSlot->setParentItem(this);
        inputSlot->setBox(this);
//        inputSlot->setFlag(QGraphicsItem::ItemIsSelectable, true);
        inputSlot->setAcceptHoverEvents(true);

        /*
         * How we compute the (vertical) positions:
         * - we want the inputs slots to be evenly spaced and centered on the box's center
         * - we define 's' the distance between each input slots' center
         * - we use integer division to correctly place the input slots
         * - in case there are an even number of inputs, we add `s/2` as offset
         */
        inputSlot->setPos(g);
        g.ry() += s;
    }

}

DiagramBox::~DiagramBox()
{
    delete m_outputSlot;
}

QRectF DiagramBox::boundingRect() const
{
    return QRectF(0, 0, bWidth + 2, bHeight + 2);
}

/*
 * Add an Arrow that originates from this Box
 */
void DiagramBox::addStartLine(Arrow *line)
{
    startLines_.insert(line);
}

/*
 * Add an Arrow that points to this Box
 */
void DiagramBox::addEndLine(Arrow *line)
{
    endLines_.insert(line);
}

/*
 * Remove the given Arrow from the list of starting lines
 */
void DiagramBox::removeStartLine(Arrow *line)
{
    startLines_.erase(startLines_.find(line));
}

/*
 * Remove the given Arrow from the list of ending lines
 */
void DiagramBox::removeEndLine(Arrow *line)
{
    endLines_.erase(endLines_.find(line));
}

/*
 * React to when the DiagramBox experiences a change
 * This is where we handle operations where a box is moved (dragged) on the scene: move its
 * connected links, etc.
 */
QVariant DiagramBox::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    // When it is moved, we need to move its connected Arrows
    if (change == QGraphicsItem::ItemPositionChange && scene()) {
        // Get coordinate of the target new position
        QPointF targetPos = value.toPointF();

        // Get the scene in order to get the grid size
        DiagramScene *theScene = qobject_cast<DiagramScene *>(scene());
        if (!theScene) {
            qFatal("ERROR: could not cast the scene into a DiagramScene!");
        }
        int gridSize = theScene->gridSize();

        // Snap the new position's (x, y) coordinates to the grid
        qreal newX = round(targetPos.x() / gridSize) * gridSize;
        qreal newY = round(targetPos.y() / gridSize) * gridSize;

        // Create the Point representing the new, snapped position
        QPointF newPos(newX, newY);

        // Compute new start and end points for the connected arrows
        QPointF newStartPoint = newPos;
        newStartPoint.rx() += boundingRect().width();
        newStartPoint.ry() += boundingRect().height() / 2;

        QPointF newEndPoint = newPos;
        newEndPoint.ry() += boundingRect().height() / 2;

        // Prompt the output slot and all inputs slots to update their connected Arrows
        m_outputSlot->updateArrows();
        foreach (InputSlot *inputSlot, m_inputSlots) {
            inputSlot->updateArrows();
        }

        // Set the script to which this item's scene is associated as modified
        theScene->script()->setStatusModified(true);

        return newPos;
    }

    return QGraphicsItem::itemChange(change, value);
}

bool DiagramBox::saveActivity() const
{
    return m_saveActivity;
}

void DiagramBox::setSaveActivity(bool saveActivity)
{
    m_saveActivity = saveActivity;
}

int DiagramBox::cols() const
{
    return m_cols;
}

void DiagramBox::setCols(int cols)
{
    if (cols < 0)
        qFatal("Cannot have a negative number of columns");

    m_cols = cols;
}

int DiagramBox::rows() const
{
    return m_rows;
}

void DiagramBox::setRows(int rows)
{
    if (rows < 0)
        qFatal("Cannot have a negative number of rows");

    m_rows = rows;
}

/**
 * @brief DiagramBox::outputType returns this function's output slot's output type, re-implemented
 * for convenience.
 * @return
 */
OutputType DiagramBox::outputType() const
{
    return m_outputSlot->outputType();
}

/**
 * @brief DiagramBox::setOutputType sets this function's output slot's output type, re-implemented
 * for convenience
 * @param outputType
 */
void DiagramBox::setOutputType(const OutputType &outputType)
{
    m_outputSlot->setOutputType(outputType);
}

std::set<InputSlot *> DiagramBox::inputSlots() const
{
    return m_inputSlots;
}

void DiagramBox::setInputSlots(const std::set<InputSlot *> &inputSlots)
{
    m_inputSlots = inputSlots;
}

/**
 * @brief DiagramBox::mousePressEvent signals the PropertiesPanel when clicked on
 * @param evt
 */
void DiagramBox::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt);
//    emit boxSelected(this);
}

OutputSlot *DiagramBox::outputSlot() const
{
    return m_outputSlot;
}

void DiagramBox::setOutputSlot(OutputSlot *outputSlot)
{
    qDebug() << "SET OUTPUT SLOT";
    m_outputSlot = outputSlot;
}

QIcon DiagramBox::icon() const
{
    return m_icon;
}

void DiagramBox::setIcon(const QIcon &icon)
{
    m_icon = icon;
}

QString DiagramBox::descriptionFile() const
{
    return m_descriptionFile;
}

void DiagramBox::setDescriptionFile(const QString &descriptionFile)
{
    m_descriptionFile = descriptionFile;
}

QUuid DiagramBox::uuid() const
{
    return m_uuid;
}

int DiagramBox::type()
{
    return DiagramBox::getType();
}

void DiagramBox::setName(const QString &name)
{
    m_name = name;
}

QString DiagramBox::name() const
{
    return m_name;
}

/**
 * @brief DiagramBox::paint draw the function box: its shape, bold when selected, function's icon,
 * function's name, etc.
 * @param painter
 * @param option
 * @param widget
 */
void DiagramBox::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);

    QPen pen;
    qreal width = 1.5;

    QFont font = painter->font();
    font.setPixelSize(13);

    // If the box is selected, make it appear bold
    if (option->state & QStyle::State_Selected) {
        width += 1;
        font.setBold(true);
    }

    pen.setWidthF(width);
    painter->setFont(font);

    QColor color = Qt::gray;
    color = color.dark();

    pen.setColor(color);

    painter->setPen(pen);

    // Draw enclosure
    painter->drawRoundedRect(QRectF(0, 0, bWidth, bHeight), 7, 7);

    // Draw vertical lines to create compartments
    painter->drawLine(QLineF(bWidth / 3, 0, bWidth / 3, bHeight - tHeight));
    painter->drawLine(QLineF(2 * bWidth / 3, 0, 2 * bWidth / 3, bHeight - tHeight));

    // Draw horizontal line to create the space for the function's name, with dashed line
    QPen oldPen = painter->pen();
    QPen dashedPen = oldPen;
    dashedPen.setStyle(Qt::DashLine);
    painter->setPen(dashedPen);
    painter->drawLine(QLineF(0, bHeight - tHeight, bWidth, bHeight - tHeight));
    painter->setPen(oldPen);

    // Draw the function's icon
    QPixmap iconPix =  m_icon.pixmap(QSize(bWidth / 3, bHeight - tHeight));
    painter->drawPixmap(QRect(bWidth / 3, 0, bWidth / 3, bHeight - tHeight), iconPix, QRect(0, 0, bWidth / 3, bHeight - tHeight));

    // Draw the function's name
    painter->drawText(QRectF(0, bHeight - tHeight, bWidth, tHeight), Qt::AlignCenter, m_name);
}

