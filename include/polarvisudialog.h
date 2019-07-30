#ifndef POLARVISUDIALOG_H
#define POLARVISUDIALOG_H

#include "visudialog.h"

class PolarVisuDialog : public VisuDialog
{
public:
	PolarVisuDialog(int maxIndex, int defaultZeroIndex, RotationDir defaultRotationDir, MatrixReadDirection defaultReadDir);
};

#endif // POLARVISUDIALOG_H
