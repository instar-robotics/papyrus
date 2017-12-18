#include "diagrambox.h"
#include "diagramscene.h"

#include <iostream>
#include <QPen>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <math.h>
#include <QDebug>

int DiagramBox::getType()
{
    return UserType + 1;
}

DiagramBox::DiagramBox(const QString &name, const QIcon &icon, OutputSlot *outputSlot, QUuid uuid, QGraphicsItem *parent) : QGraphicsItem(parent),
                                                m_uuid(uuid),
                                                m_name(name),
                                                m_icon(icon),
                                                startLines_(),
                                                endLines_(),
                                                bWidth(175),
                                                bHeight(70),
                                                tHeight(20),
                                                m_outputSlot(outputSlot)
{
    // Generate a UUID if there was not one while created
    if (m_uuid.isNull())
        m_uuid = QUuid::createUuid();

    setFlags(QGraphicsItem::ItemIsSelectable
           | QGraphicsItem::ItemIsMovable
           | QGraphicsItem::ItemSendsScenePositionChanges);
    setAcceptHoverEvents(true);

    // Make this the parent item of the output slot, so that it follow drags, etc.
    outputSlot->setParentItem(this);

    // Set the output's slot position, in its parent's referential (this item's)
    QPointF p = (boundingRect().bottomRight() + boundingRect().topRight()) / 2;
    p.ry() -= outputSlot->boundingRect().height() / 2;
    p.rx() += 5; // Set a bit of margin to the right to prevent the diamon-shape to overlap
    outputSlot->setPos(p);
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
 */
QVariant DiagramBox::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    // When it is moved, we need to move its connected Arrows
    if (change == QGraphicsItem::ItemPositionChange && scene()) {
        // Get coordinate of the target new position
        QPointF targetPos = value.toPointF();

        // Get the scene in order to get the grid size
        DiagramScene *theScene = qobject_cast<DiagramScene *>(scene());
        int gridSize = theScene->gridSize();

        // Snap the new position's (x, y) coordinates to the grid
        qreal newX = round(targetPos.x() / gridSize) * gridSize;
        qreal newY = round(targetPos.y() / gridSize) * gridSize;

        // Create the Point representing the new, snapped position
        QPointF newPos(newX, newY);

        QPointF newStartPoint = newPos;
        newStartPoint.rx() += boundingRect().width();
        newStartPoint.ry() += boundingRect().height() / 2;

        QPointF newEndPoint = newPos;
        newEndPoint.ry() += boundingRect().height() / 2;

        // Update position of start lines
        for (auto line : startLines_) {
            line->updatePosition(newStartPoint, true);
        }

        // Update position of end lines
        for (auto line : endLines_) {
            line->updatePosition(newEndPoint, false);
        }

        // Set the script to which this item's scene is associated as modified
        theScene->script()->setStatusModified(true);

        return newPos;
    } else if (change == QGraphicsItem::ItemSelectedHasChanged && isSelected()) {
        std::cout << std::endl;
        std::cout << "Box #??? selected, it has " << startLines().size()
                  << " start lines and " << endLines().size() << " end lines." << std::endl;
    }

    return QGraphicsItem::itemChange(change, value);
}

OutputSlot *DiagramBox::outputSlot() const
{
    return m_outputSlot;
}

void DiagramBox::setOutputSlot(OutputSlot *outputSlot)
{
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

