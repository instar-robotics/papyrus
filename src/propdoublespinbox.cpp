#include "propdoublespinbox.h"

PropDoubleSpinBox::PropDoubleSpinBox(QWidget *parent) : QDoubleSpinBox(parent),
    m_sizeHint(QSize(176, 44))
{

}

QSize PropDoubleSpinBox::sizeHint() const
{
	return m_sizeHint;
}

void PropDoubleSpinBox::setSizeHint(const QSize &sizeHint)
{
	m_sizeHint = sizeHint;
}
