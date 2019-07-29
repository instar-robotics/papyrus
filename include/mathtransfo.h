#ifndef MATHTRANSFO_H
#define MATHTRANSFO_H

#include <QMatrix3x3>
#include <QVector3D>
#include <math.h>

const float MAP_SIZE = 5.0f;
const double PI = 3.141592653589793238463;

float radToDeg(float x);
float degToRad(float x);
float normalizeDeg(float x);

QVector3D vecRotationX(QVector3D vector, float angle);
QVector3D vecRotationY(QVector3D vector, float angle);
QVector3D vecRotationZ(QVector3D vector, float angle);
QVector3D multMatrix3x3ByVec(QMatrix3x3 mat, QVector3D vec);


#endif // MATHTRANSFO_H
