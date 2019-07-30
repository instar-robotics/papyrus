#include "circularvisudialog.h"

CircularVisuDialog::CircularVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir)
{
	choseRotationDirection(defaultRotationDir);
	choseIndexOfZeroAngle(maxIndex, defaultZeroIndex);
	addCloseButton();
}
