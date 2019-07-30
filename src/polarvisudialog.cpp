#include "polarvisudialog.h"

PolarVisuDialog::PolarVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, MatrixReadDirection defaultReadDir)
{
	choseRotationDirection(defaultRotationDir);
	choseIndexOfZeroAngle(maxIndex, defaultZeroIndex);
	choseMatrixReadDirection(defaultReadDir);
	addCloseButton();
}
