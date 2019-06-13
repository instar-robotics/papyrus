#ifndef SHADERMOVEBAR_H
#define SHADERMOVEBAR_H

#include <QGraphicsRectItem>

class ShaderProxy;

class ShaderMoveBar : public QGraphicsRectItem
{
public:
	ShaderMoveBar();
	~ShaderMoveBar();

	ShaderProxy *proxy() const;
	void setProxy(ShaderProxy *proxy);

private:
	ShaderProxy *m_proxy;
};

#endif // SHADERMOVEBAR_H
