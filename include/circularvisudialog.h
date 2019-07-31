#ifndef CIRCULARVISUDIALOG_H
#define CIRCULARVISUDIALOG_H

#include "visudialog.h"

class CircularVisuDialog : public VisuDialog
{
public:
	CircularVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, int defaultExtremum);
};

#endif // CIRCULARVISUDIALOG_H
