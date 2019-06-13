#include "shadermovebar.h"

#include <QDebug>

ShaderMoveBar::ShaderMoveBar()
{

}

ShaderMoveBar::~ShaderMoveBar()
{
	m_proxy = nullptr;
}

ShaderProxy* ShaderMoveBar::proxy() const
{
	return m_proxy;
}

void ShaderMoveBar::setProxy(ShaderProxy *proxy)
{
	m_proxy = proxy;
}
