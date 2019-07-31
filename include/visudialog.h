﻿#ifndef VISUDIALOG_H
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
	int getExtremum();

protected:
	void choseRotationDirection(RotationDir defaultRotationDir);
	void choseIndexOfZeroAngle(int maxIndex, int defaultZeroIndex);
	void choseMatrixReadDirection(MatrixReadDirection defaultReadDir);
	void choseDomainOfDefinition(int defaultExtremum, int minimum, int maximum);
	void addCloseButton();

public slots:
	void closeDialog();

protected:
	QVBoxLayout *m_layout;
	QPushButton *m_closeButton;

	//Matrix read direction
	QLabel *m_matrixReadDirLabel;
	QVBoxLayout *m_matrixReadRadioBoxLayout;
	QGroupBox *m_matrixReadRadioBox;
	QRadioButton *m_linePerLine;
	QRadioButton *m_columnPerColumn;

	//Rotation direction
	QLabel *m_rotationDirLabel;
	QVBoxLayout *m_rotationRadioBoxLayout;
	QGroupBox *m_rotationRadioBox;
	QRadioButton *m_clockwise;
	QRadioButton *m_counterclockwise;

	//Index of the 0 angle
	QLabel *m_indexLabel;
	QSpinBox *m_zeroIndex;

	//Extremum of the domain of definition
	QLabel *m_domainLabel;
	QSpinBox *m_extremum;
};

#endif // VISUDIALOG_H
