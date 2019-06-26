#ifndef SCOPEWINDOW_H
#define SCOPEWINDOW_H

#include "script.h"

#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>

namespace Ui {
class ScopeWindow;
}

/**
 * @brief The ScopeWindow class is a window in which the scope for the associated script is
 * displayed.
 */

class ScopeWindow : public QDialog
{
	Q_OBJECT

public:
	explicit ScopeWindow(Script *script, QWidget *parent = 0);
	~ScopeWindow();

	void setTitle(const QString &title);
	void initAxes();
	void initTicks();

	void enableOscillo();
	void disableOscillo();

private:
	Ui::ScopeWindow *m_ui;
	Script *m_script;

	qreal m_xOffset;
	qreal m_yOffset;

	QGraphicsScene m_scene;
	QGraphicsLineItem m_timeAxis;
	QGraphicsLineItem m_functionsAxis;
	int m_nbTimeTicks;                    // The number of ticks on the time (x) axis
	QList<QGraphicsLineItem *> m_timeTicks; // The ticks on the time axis
	QList<QGraphicsTextItem *> m_timeTicksLabels;

};

#endif // SCOPEWINDOW_H
