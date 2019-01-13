#include "proplineedit.h"

PropLineEdit::PropLineEdit(QWidget *parent) : QLineEdit(parent), m_sizeHint(QSize(270, 35))
{

}

PropLineEdit::PropLineEdit(const QString &contents, QWidget *parent) : QLineEdit(contents, parent), m_sizeHint(QSize(270, 35))
{

}

QSize PropLineEdit::sizeHint() const
{
	return m_sizeHint;
}

void PropLineEdit::setSizeHint(const QSize &sizeHint)
{
	m_sizeHint = sizeHint;
}
