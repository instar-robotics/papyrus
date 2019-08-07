#ifndef CIRCULARVISUDIALOG_H
#define CIRCULARVISUDIALOG_H

#include "visudialog.h"

/**
 * @brief The CircularVisuDialog class is the QDialog used to provide parameters menu for every circular visu.
 */

class CircularVisuDialog : public VisuDialog
{
public:
	CircularVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, int defaultExtremum);
};

#endif // CIRCULARVISUDIALOG_H
