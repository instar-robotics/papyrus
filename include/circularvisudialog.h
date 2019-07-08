#ifndef CIRCULARVISUDIALOG_H
#define CIRCULARVISUDIALOG_H

#include <QDebug>
#include <QDialog>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include "types.h"

class CircularVisuDialog : public QDialog
{
	Q_OBJECT
public:
	CircularVisuDialog(int maxIndex, int defaultZeroIndex);
	int getZeroIndex();
	RotationDir getRotationDirection();

public slots:
	void closeDialog();

private:
	QVBoxLayout *m_layout;
	QVBoxLayout *m_radioBoxLayout;
	QGroupBox *m_radioBox;
	QRadioButton *m_clockwise;
	QRadioButton *m_counterclockwise;
	QSpinBox *m_zeroIndex;
	QLabel *m_rotationDirLayout;
	QLabel *m_indexLabel;
	QPushButton *m_closeButton;
};

#endif // CIRCULARVISUDIALOG_H
