#ifndef PROPDOUBLESPINBOX_H
#define PROPDOUBLESPINBOX_H

#include <QDoubleSpinBox>

/**
 * @brief The PropDoubleSpinBox class is the standard QDoubleSpinBox with only one minor
 * modification: its sizeHint has been made editable
 */

class PropDoubleSpinBox : public QDoubleSpinBox
{
public:
	PropDoubleSpinBox(QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint;
};

#endif // PROPDOUBLESPINBOX_H
