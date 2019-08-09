#ifndef POLARVISUDIALOG_H
#define POLARVISUDIALOG_H

#include "visuparamdialog.h"

/**
 * @brief The PolarVisuDialog class is the QDialog used to provide parameters menu for every polar visu.
 */

class PolarVisuParamDialog : public VisuParamDialog
{
public:
	PolarVisuParamDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, MatrixReadDirection defaultReadDir, int defaultExtremum);
};

#endif // POLARVISUDIALOG_H
