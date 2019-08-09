#ifndef VISUPARAMFUNCTIONS_H
#define VISUPARAMFUNCTIONS_H

#include "shaderbarchart.h"
#include "shaderconechart.h"
#include "shadersurface.h"
#include "shaderwireframe.h"
#include "shadercrown.h"
#include "shaderbarcircle.h"
#include "shaderbarpolar.h"
#include "shaderwireframepolar.h"
#include "shadersurfacepolar.h"
#include "circularvisuparamdialog.h"
#include "shadercompass.h"

VisuType stringToVisuType(const QString &str);

QString visuTypeToString(const VisuType &visuType);

bool is2DVisuType(const VisuType &visuType);

bool is3DVisuType(const VisuType &visuType);

bool is3DMatrixVisuType(const VisuType &visuType);

bool is3DCircularVisuType(const VisuType &visuType);

bool is3DPolarVisuType(const VisuType &visuType);

ShaderWidget* createShaderWidget(VisuType type, int rows, int cols, QMap<QString, QVariant> parameters);

RotationDir intToRotationDir(int rotDir);

bool doesVisuFit(VisuType type, int rows, int cols); //Test if rows and cols dimensions fit with the visu type

#endif // VISUPARAMFUNCTIONS_H
