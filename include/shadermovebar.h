#ifndef SHADERMOVEBAR_H
#define SHADERMOVEBAR_H

#include <QGraphicsRectItem>

class ShaderProxy;
class ThreadShader;

class ShaderMoveBar : public QGraphicsRectItem
{
public:
	ShaderMoveBar();
	~ShaderMoveBar();

	ShaderProxy *proxy() const;
	void setProxy(ShaderProxy *proxy);

	ThreadShader *thread() const;

	void setThread(ThreadShader *thread);

private:
	ShaderProxy *m_proxy;
	ThreadShader *m_thread;
};

#endif // SHADERMOVEBAR_H
