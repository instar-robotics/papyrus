#ifndef VISUDIALOG_H
#define VISUDIALOG_H

#include <QDebug>
#include <QDialog>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include "types.h"

class VisuDialog : public QDialog
{
	Q_OBJECT
public:
	VisuDialog();
	int getZeroIndex();
	RotationDir getRotationDirection();
	MatrixReadDirection getMatrixReadDirection();

protected:
	void choseRotationDirection(RotationDir defaultRotationDir);
	void choseIndexOfZeroAngle(int maxIndex, int defaultZeroIndex);
	void choseMatrixReadDirection(MatrixReadDirection defaultReadDir);
	void addCloseButton();

public slots:
	void closeDialog();

protected:
	QVBoxLayout *m_layout;
	QPushButton *m_closeButton;

	QLabel *m_matrixReadDirLabel;
	QVBoxLayout *m_matrixReadRadioBoxLayout;
	QGroupBox *m_matrixReadRadioBox;
	QRadioButton *m_linePerLine;
	QRadioButton *m_columnPerColumn;

	QLabel *m_rotationDirLabel;
	QVBoxLayout *m_rotationRadioBoxLayout;
	QGroupBox *m_rotationRadioBox;
	QRadioButton *m_clockwise;
	QRadioButton *m_counterclockwise;

	QSpinBox *m_zeroIndex;
	QLabel *m_indexLabel;
};

#endif // VISUDIALOG_H
