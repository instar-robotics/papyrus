#ifndef SCOPEWINDOW_H
#define SCOPEWINDOW_H

#include "script.h"
#include "scopemessage.h"
#include "scopeitem.h"

#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QGraphicsRectItem>
#include <QMap>

Q_DECLARE_METATYPE(hieroglyph::OscilloArray::ConstPtr)

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

	void showEvent(QShowEvent *evt);

	void setTitle(const QString &title);
	void initAxes();
	void initTicks();
	void initScopeItems();

	void enableOscillo();
	bool disableOscillo();

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

	qreal m_period; // Store the period in ms
	QMap<QUuid, ScopeItem *> m_scopeItems; // The set of rectangles to draw duration, referenced by uuid
	QMap<QUuid, QGraphicsTextItem *> m_scopeLabels; // The matching between Uuid and the name/title of a function

private slots:
	void onNewOscilloMessage(QVector<ScopeMessage> *scopeMessages);
};

#endif // SCOPEWINDOW_H
