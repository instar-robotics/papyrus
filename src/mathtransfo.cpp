#include "mathtransfo.h"

MathTransfo::MathTransfo()
{

}

float MathTransfo::radToDeg(float x)
{
	return x*180/PI;
}

float MathTransfo::degToRad(float x)
{
	return x/180*PI;
}
