#ifndef SETCOLORBUTTON_H
#define SETCOLORBUTTON_H

#include <QPushButton>
#include <QColor>

class SetColorButton : public QPushButton
{
	Q_OBJECT

public:
	explicit SetColorButton(QWidget *parent = nullptr);

	void updateColor();

	QColor color() const;
	void setColor(const QColor &color);

private:
	QColor m_color;

private slots:
	void changeColor();
};

#endif // SETCOLORBUTTON_H
