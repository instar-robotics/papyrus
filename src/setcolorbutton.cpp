#include "setcolorbutton.h"

#include <QColorDialog>
#include <QDebug>

SetColorButton::SetColorButton(QWidget *parent) : QPushButton(parent), m_color(qRgba(51, 153, 255, 10))
{
	connect(this, SIGNAL(clicked()), this, SLOT(changeColor()));
}

void SetColorButton::updateColor()
{
	setStyleSheet("background-color: " + m_color.name());
}

QColor SetColorButton::color() const
{
	return m_color;
}

void SetColorButton::setColor(const QColor &color)
{
	m_color = color;
	updateColor();
}

void SetColorButton::changeColor()
{
	QColor newColor = QColorDialog::getColor(m_color,
	                                         parentWidget(),
	                                         tr("Change comment zone color"),
	                                         QColorDialog::ShowAlphaChannel);
	if (newColor != m_color)
		setColor(newColor);
}
