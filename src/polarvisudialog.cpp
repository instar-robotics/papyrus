#include "polarvisudialog.h"

PolarVisuDialog::PolarVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir,
                                 MatrixReadDirection defaultReadDir, int defaultExtremum)
{
	choseMatrixReadDirection(defaultReadDir);
	choseRotationDirection(defaultRotationDir);
	choseIndexOfZeroAngle(maxIndex, defaultZeroIndex);
	choseDomainOfDefinition(defaultExtremum,1,360);
	addCloseButton();
}
