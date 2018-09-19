#ifndef CONSTANTDIAGRAMBOX_H
#define CONSTANTDIAGRAMBOX_H

#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"

#include <QGraphicsItem>
#include <QIcon>
#include <QUuid>
#include <QPainter>

class ConstantDiagramBox : public DiagramBox
{
	Q_OBJECT

public:
	explicit ConstantDiagramBox(const QString &name,
	                            const QIcon &icon,
	                            OutputSlot *outputSlot,
	                            const QUuid &uuid = 0,
	                            QGraphicsItem *parent = 0);

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // CONSTANTDIAGRAMBOX_H
