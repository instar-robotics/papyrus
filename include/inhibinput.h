#ifndef INHIBINPUT_H
#define INHIBINPUT_H

#include "inputslot.h"
#include <QGraphicsLineItem>

class InhibInput : public InputSlot
{
public:
	explicit InhibInput();

	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
	QGraphicsLineItem m_line;
};

#endif // INHIBINPUT_H
