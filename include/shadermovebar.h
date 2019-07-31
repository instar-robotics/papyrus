#ifndef SHADERMOVEBAR_H
#define SHADERMOVEBAR_H

#include <QGraphicsRectItem>
#include <QPainter>
#include <QBrush>
#include <QFont>

class ShaderProxy;

class ShaderMoveBar : public QGraphicsRectItem
{
public:
	ShaderMoveBar();
	~ShaderMoveBar();

	ShaderProxy *proxy() const;
	void setProxy(ShaderProxy *proxy);

	void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget) override;

	void setTitle(const QString &title);

private:
	ShaderProxy *m_proxy;
	QString m_title;
	int m_fontSize = 12;
};

#endif // SHADERMOVEBAR_H
