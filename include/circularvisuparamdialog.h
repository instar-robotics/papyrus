#ifndef CIRCULARVISUPARAMDIALOG_H
#define CIRCULARVISUPARAMDIALOG_H

#include "visuparamdialog.h"

/**
 * @brief The CircularVisuDialog class is the QDialog used to provide parameters menu for every circular visu.
 */

class CircularVisuParamDialog : public VisuParamDialog
{
public:
	CircularVisuParamDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, int defaultExtremum);
};

#endif // CIRCULARVISUPARAMDIALOG_H
