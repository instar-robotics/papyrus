#include "circularvisudialog.h"

CircularVisuDialog::CircularVisuDialog(int maxIndex, int defaultZeroIndex)
{
	setWindowTitle("Data needed");
	m_layout = new QVBoxLayout();
	setLayout(m_layout);

	//Chose the rotation direction
	m_rotationDirLayout = new QLabel("Rotation direction:");
	m_radioBox = new QGroupBox(this);
	m_clockwise = new QRadioButton("Clockwise");
	m_counterclockwise = new QRadioButton("Counterclockwise");
	m_clockwise->setChecked(true);
	m_radioBoxLayout = new QVBoxLayout();
	m_radioBox->setLayout(m_radioBoxLayout);

	m_radioBoxLayout->addWidget(m_clockwise);
	m_radioBoxLayout->addWidget(m_counterclockwise);

	//Chose the index of 0 angle
	m_indexLabel = new QLabel("Index of the 0 angle:");
	m_zeroIndex = new QSpinBox(this);
	m_zeroIndex->setMinimum(0);
	m_zeroIndex->setMaximum(maxIndex);
	m_zeroIndex->setValue(defaultZeroIndex);

	//Close dialog button
	m_closeButton = new QPushButton("Ok");
	//connect(m_closeButton, SIGNAL(clicked()), this, SLOT(closeDialog()));
	connect(m_closeButton, SIGNAL(clicked()), this, SLOT(accept()));

	m_layout->addWidget(m_rotationDirLayout);
	m_layout->addWidget(m_radioBox);
	m_layout->addWidget(m_indexLabel);
	m_layout->addWidget(m_zeroIndex);
	m_layout->addWidget(m_closeButton);
}

int CircularVisuDialog::getZeroIndex()
{
	return m_zeroIndex->value();
}
RotationDir CircularVisuDialog::getRotationDirection()
{
	if(m_clockwise->isChecked())
		return CLOCKWISE;
	else
		return COUNTERCLOCKWISE;
}

void CircularVisuDialog::closeDialog()
{
	close();
}
