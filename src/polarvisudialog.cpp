#include "polarvisudialog.h"

PolarVisuDialog::PolarVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir,
                                 MatrixReadDirection defaultReadDir, int extremum)
{
	choseRotationDirection(defaultRotationDir);
	choseIndexOfZeroAngle(maxIndex, defaultZeroIndex);
	choseMatrixReadDirection(defaultReadDir);
	choseDomainOfDefinition(extremum,1,360);
	addCloseButton();
}
