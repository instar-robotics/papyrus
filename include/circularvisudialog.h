#ifndef CIRCULARVISUDIALOG_H
#define CIRCULARVISUDIALOG_H

#include "visudialog.h"

class CircularVisuDialog : public VisuDialog
{
public:
	CircularVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir);
};

#endif // CIRCULARVISUDIALOG_H
