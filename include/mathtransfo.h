#ifndef MATHTRANSFO_H
#define MATHTRANSFO_H

const float MAP_SIZE = 5.0f;
const double PI = 3.141592653589793238463;

class MathTransfo
{
public:
	MathTransfo();

	static float radToDeg(float x);
	static float degToRad(float x);
	static float normalizeDeg(float x);
};

#endif // MATHTRANSFO_H
