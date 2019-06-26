#include "scopewindow.h"
#include "ui_scopewindow.h"

#include <QDebug>
#include <QScreen>
#include <QGraphicsRectItem>

ScopeWindow::ScopeWindow(Script *script, QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::ScopeWindow),
    m_script(script),
    m_xOffset(150),
    m_yOffset(25),
    m_scene(this),
    m_nbTimeTicks(10)
{
	m_ui->setupUi(this);

	// Resize window
	QScreen *screen = QGuiApplication::primaryScreen();
	QSize screenSize = screen->availableSize();
	resize(screenSize.width() - 200,
	       screenSize.height() / 2.0);

	// Enable oscillo on the kernel
	enableOscillo();

	m_ui->graphicsView->setScene(&m_scene);
	m_scene.setBackgroundBrush(QColor(Qt::gray).darker());
	m_scene.setSceneRect(0, 0, 1650, 400);

	initAxes();
	initTicks();
}

ScopeWindow::~ScopeWindow()
{
	disableOscillo();

	delete m_ui;
}

void ScopeWindow::setTitle(const QString &title)
{
	m_ui->label->setText(title);
}

/**
 * @brief ScopeWindow::initAxes draws the axes for the scope display
 */
void ScopeWindow::initAxes()
{
	m_timeAxis.setLine(0, m_yOffset, m_scene.width(), m_yOffset);
	m_timeAxis.setPen(QPen(Qt::white));
	m_scene.addItem(&m_timeAxis);

	m_functionsAxis.setLine(m_xOffset, 0, m_xOffset, m_scene.height());
	m_functionsAxis.setPen(QPen(Qt::white));
	m_scene.addItem(&m_functionsAxis);
}

/**
 * @brief ScopeWindow::initTicks create the ticks and labels of the scope display
 */
void ScopeWindow::initTicks()
{
	if (m_script == nullptr) {
		qWarning() << "[ScopeWindow::initTicks] no script!";
		return;
	}

	qreal dx = (m_scene.width() - m_xOffset) / m_nbTimeTicks;

	for (int i = 0; i < m_nbTimeTicks; i += 1) {
		qreal x1 = m_xOffset + (i+1) * dx;
		qreal y1 = m_yOffset;
		qreal x2 = x1;
		qreal y2 = y1 - 5;
		QGraphicsLineItem *tick = new QGraphicsLineItem(x1, y1, x2, y2);
		tick->setPen(QPen(Qt::white));
		m_scene.addItem(tick);
		m_timeTicks << tick;

		qreal period;

		if (m_script->timeUnit() == HZ)
			period = 1000.0 / m_script->timeValue();
		else
			period = m_script->timeValue();

		QString value = QString::number((i+1) * period / m_nbTimeTicks, 'g', 4);
		QGraphicsTextItem *label = new QGraphicsTextItem;
		label->setHtml(QString("<center>%1ms</center>").arg(value));
		QFont f = label->font();
		f.setPointSizeF(f.pointSizeF() - 2.5);
		label->setFont(f);
		label->setDefaultTextColor(Qt::white);
		label->setTextWidth(dx);
		label->setPos(x2 - dx / 2.0, y2 - 25);
		m_scene.addItem(label);
		m_timeTicksLabels << label;
	}
}

/**
 * @brief ScopeWindow::enableOscillo calls the "oscillo" ROSService of the associated script to
 * start oscillo.
 */
void ScopeWindow::enableOscillo()
{
	if (m_script != nullptr) {
		if (m_script->rosSession() != nullptr) {
			m_script->rosSession()->callServiceOscillo("start");
		} else {
			qWarning() << "[ScopeWindow::enableOscillo] cannot enable oscillo: no ROSSession!";
		}
	} else {
		qWarning() << "[ScopeWindow::enableOscillo] ScopeWindow created without a Script!";
	}
}

/**
 * @brief ScopeWindow::disableOscillo calls the "oscillo" ROSService of the associated script to
 * stop oscillo.
 */
void ScopeWindow::disableOscillo()
{
	if (m_script != nullptr) {
		if (m_script->rosSession() != nullptr) {
			m_script->rosSession()->callServiceOscillo("stop");
		} else {
			qWarning() << "[ScopeWindow::enableOscillo] cannot enable oscillo: no ROSSession!";
		}
	} else {
		qWarning() << "[ScopeWindow::enableOscillo] ScopeWindow created without a Script!";
	}
}
