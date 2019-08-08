﻿#include "visudialog.h"

VisuDialog::VisuDialog()
{
	setWindowTitle("Visualization parameters");
	m_layout = new QVBoxLayout();
	setLayout(m_layout);
}

int VisuDialog::getZeroIndex()
{
	return m_zeroIndex->value();
}
RotationDir VisuDialog::getRotationDirection()
{
	if(m_clockwise->isChecked())
		return CLOCKWISE;
	else
		return COUNTERCLOCKWISE;
}
MatrixReadDirection VisuDialog::getMatrixReadDirection()
{
	if(m_linePerLine->isChecked())
		return LINE_PER_LINE;
	else
		return COLUMN_PER_COLUMN;
}
int VisuDialog::getExtremum()
{
	return m_extremum->value();
}

//Chose the rotation direction
void VisuDialog::choseRotationDirection(RotationDir defaultRotationDir)
{
	m_rotationDirLabel = new QLabel("Rotation direction:");
	m_rotationRadioBox = new QGroupBox(this);
	m_rotationRadioBoxLayout = new QVBoxLayout();
	m_clockwise = new QRadioButton("Clockwise");
	m_counterclockwise = new QRadioButton("Counterclockwise");
	if(defaultRotationDir == CLOCKWISE)
		m_clockwise->setChecked(true);
	else
		m_counterclockwise->setChecked(true);
	m_rotationRadioBox->setLayout(m_rotationRadioBoxLayout);

	m_rotationRadioBoxLayout->addWidget(m_clockwise);
	m_rotationRadioBoxLayout->addWidget(m_counterclockwise);

	m_layout->addWidget(m_rotationDirLabel);
	m_layout->addWidget(m_rotationRadioBox);
}

//Chose the index of 0 angle
void VisuDialog::choseIndexOfZeroAngle(int maxIndex, int defaultZeroIndex)
{
	m_indexLabel = new QLabel("Index of the 0 angle:");
	m_zeroIndex = new QSpinBox(this);
	m_zeroIndex->setMinimum(0);
	m_zeroIndex->setMaximum(maxIndex);
	m_zeroIndex->setValue(defaultZeroIndex);

	m_layout->addWidget(m_indexLabel);
	m_layout->addWidget(m_zeroIndex);
}

void VisuDialog::choseMatrixReadDirection(MatrixReadDirection defaultReadDir)
{
	m_matrixReadDirLabel = new QLabel("Rotation direction:");
	m_matrixReadRadioBox = new QGroupBox(this);
	m_matrixReadRadioBoxLayout = new QVBoxLayout();
	m_linePerLine = new QRadioButton("Line per line");
	m_columnPerColumn = new QRadioButton("Column per column");
	if(defaultReadDir == LINE_PER_LINE)
		m_linePerLine->setChecked(true);
	else
		m_columnPerColumn->setChecked(true);
	m_matrixReadRadioBox->setLayout(m_matrixReadRadioBoxLayout);

	m_matrixReadRadioBoxLayout->addWidget(m_linePerLine);
	m_matrixReadRadioBoxLayout->addWidget(m_columnPerColumn);

	m_layout->addWidget(m_matrixReadDirLabel);
	m_layout->addWidget(m_matrixReadRadioBox);
}

void VisuDialog::choseDomainOfDefinition(int defaultExtremum, int minimum, int maximum)
{
	m_domainLabel = new QLabel("Extremum of the domain of definition:");
	m_extremum = new QSpinBox(this);

	m_extremum->setMinimum(minimum);
	m_extremum->setMaximum(maximum);
	m_extremum->setValue(defaultExtremum);

	m_layout->addWidget(m_domainLabel);
	m_layout->addWidget(m_extremum);
}

void VisuDialog::addCloseButton()
{
	//Close dialog button
	m_closeButton = new QPushButton("Ok");
	connect(m_closeButton, SIGNAL(clicked()), this, SLOT(accept()));

	m_layout->addWidget(m_closeButton);
}

void VisuDialog::closeDialog()
{
	close();
}
