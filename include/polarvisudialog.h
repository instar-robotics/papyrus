#ifndef POLARVISUDIALOG_H
#define POLARVISUDIALOG_H

#include "visudialog.h"

/**
 * @brief The PolarVisuDialog class is the QDialog used to provide parameters menu for every polar visu.
 */

class PolarVisuDialog : public VisuDialog
{
public:
	PolarVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, MatrixReadDirection defaultReadDir, int defaultExtremum);
};

#endif // POLARVISUDIALOG_H
