#ifndef VISUFUNCTIONS_H
#define VISUFUNCTIONS_H

#include "shaderbarchart.h"
#include "shaderconechart.h"
#include "shadersurface.h"
#include "shaderwireframe.h"
#include "shadercrown.h"
#include "shaderbarcircle.h"
#include "shaderbarpolar.h"
#include "shaderwireframepolar.h"
#include "shadersurfacepolar.h"
#include "circularvisudialog.h"

VisuType stringToVisuType(const QString &str);

QString visuTypeToString(const VisuType &visuType);

bool is2DVisuType(const VisuType &visuType);

bool is3DVisuType(const VisuType &visuType);

bool is3DMatrixVisuType(const VisuType &visuType);

bool is3DCircularVisuType(const VisuType &visuType);

bool is3DPolarVisuType(const VisuType &visuType);

ShaderWidget* createShaderWidget(VisuType type, int rows, int cols, QVector<QVariant> parameters);

RotationDir intToRotationDir(int rotDir);

void copyVisuParameters(QVector<QVariant> origin, QVector<QVariant>* destination);

#endif // VISUFUNCTIONS_H
