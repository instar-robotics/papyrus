#include "circularvisuparamdialog.h"

CircularVisuParamDialog::CircularVisuParamDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, int defaultExtremum)
{
	choseRotationDirection(defaultRotationDir);
	choseIndexOfZeroAngle(maxIndex, defaultZeroIndex);
	choseDomainOfDefinition(defaultExtremum, 1, 360);
	addCloseButton();
}
