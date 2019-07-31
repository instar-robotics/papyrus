#include "polarvisudialog.h"

PolarVisuDialog::PolarVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir,
                                 MatrixReadDirection defaultReadDir, int defaultExtremum)
{
	choseRotationDirection(defaultRotationDir);
	choseIndexOfZeroAngle(maxIndex, defaultZeroIndex);
	choseMatrixReadDirection(defaultReadDir);
	choseDomainOfDefinition(defaultExtremum,1,360);
	addCloseButton();
}
