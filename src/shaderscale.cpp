#include "shaderscale.h"

ShaderScale::ShaderScale(float range, int nbMeasuresY)
{
	m_range = range;
	m_nbMeasuresY = nbMeasuresY;
	m_measureY = 2*m_range/(m_nbMeasuresY-1);
}

ShaderScale::~ShaderScale()
{
}

int ShaderScale::nbMeasuresY() const
{
	return m_nbMeasuresY;
}
