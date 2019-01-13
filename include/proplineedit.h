#ifndef PROPLINEEDIT_H
#define PROPLINEEDIT_H

#include <QLineEdit>

/**
 * @brief The PropLineEdit class is a standard @QLineEdit with only one minor modification: its
 * sizeHint can be changed
 */

class PropLineEdit : public QLineEdit
{
public:
	PropLineEdit(QWidget *parent = nullptr);
	PropLineEdit(const QString &contents, QWidget *parent = nullptr);

	QSize sizeHint() const override;
	void setSizeHint(const QSize &sizeHint);

private:
	QSize m_sizeHint; // Modified sizeHint
};

#endif // PROPLINEEDIT_H
