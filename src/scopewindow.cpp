#include "scopewindow.h"
#include "ui_scopewindow.h"
#include "helpers.h"
#include "diagramscene.h"
#include "diagrambox.h"
#include "constantdiagrambox.h"

#include <QDebug>
#include <QScreen>
#include <QGraphicsItem>
#include <QScrollBar>

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

	// Init period based on the script
	if (m_script->timeUnit() == HZ)
		m_period = 1000.0 / m_script->timeValue();
	else
		m_period = m_script->timeValue();

	// Init scene
	initScopeItems();

	// Set scene' background
	m_scene.setBackgroundBrush(QColor(0, 59, 0));
	m_scene.setSceneRect(0, 0, 1650, qMax(500.0, m_scopeItems.count() * (ScopeItem::barHeight)));
	initAxes();
	initTicks();
	m_ui->graphicsView->setScene(&m_scene);

	connect(m_script->rosSession(), SIGNAL(newOscilloMessage(QVector<ScopeMessage>*)),
	        this, SLOT(onNewOscilloMessage(QVector<ScopeMessage>*)));
}

ScopeWindow::~ScopeWindow()
{
	disableOscillo();

	delete m_ui;
}

void ScopeWindow::showEvent(QShowEvent *evt)
{
	// Scroll at the top when the widget is displayed
	m_ui->graphicsView->verticalScrollBar()->setValue(0);

	QDialog::showEvent(evt);
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
	if (m_script == nullptr) {
		qWarning() << "[ScopeWindow::initAxes] no script!";
		return;
	}

	m_timeAxis.setLine(0, m_yOffset, m_scene.width(), m_yOffset);
	m_timeAxis.setPen(QPen(Qt::white));
	m_scene.addItem(&m_timeAxis);

	m_functionsAxis.setLine(m_xOffset, 0, m_xOffset, m_scene.height());
	m_functionsAxis.setPen(QPen(Qt::white));
	m_scene.addItem(&m_functionsAxis);

	// Draw a line separator between the scope items
	int n = m_scopeItems.size();
	qreal itemHeight = ScopeItem::barHeight;

	for (int i = 0; i < n; i += 1) {
		m_scene.addLine(0,
		                m_yOffset - 1 + (i+1) * (itemHeight + 5),
		                m_scene.width(),
		                m_yOffset - 1 + (i+1) * (itemHeight + 5),
		                QPen(Qt::gray)
		                );
	}
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

		QString value = QString::number((i+1) * m_period / m_nbTimeTicks, 'g', 4);
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
 * @brief ScopeWindow::initRects parse the script and create a @QGraphicsRectItem per @DiagramBox
 */
void ScopeWindow::initScopeItems()
{
	if (m_script == nullptr) {
		qWarning() << "[ScopeWindow::initRects] no script!";
		return;
	}

	if (m_script->scene() == nullptr) {
		qWarning() << "[ScopeWindow::initRects] script has no scene!";
		return;
	}

	foreach(QGraphicsItem *item, m_script->scene()->items()) {
		DiagramBox *maybeBox = dynamic_cast<DiagramBox *>(item);
		ConstantDiagramBox *maybeConstant = dynamic_cast<ConstantDiagramBox *>(item);

		if (maybeBox != nullptr && maybeConstant == nullptr) {
			ScopeItem *scopeItem = new ScopeItem;
			m_scopeItems.insert(maybeBox->uuid(), scopeItem);
			m_scene.addItem(scopeItem);

			QGraphicsTextItem *label = new QGraphicsTextItem;
			label->setHtml(QString("%1").arg(maybeBox->title().isEmpty() ? maybeBox->name() : maybeBox->title()));
			QFont f = label->font();
			f.setPointSizeF(f.pointSizeF() - 3.5);
			label->setFont(f);
			label->setDefaultTextColor(Qt::white);
			label->setTextWidth(m_xOffset);

			m_scopeLabels.insert(maybeBox->uuid(),
			                     label);
			m_scene.addItem(label);
		}
	}
}

/**
 * @brief ScopeWindow::enableOscillo calls the "oscillo" ROSService of the associated script to
 * start oscillo. It also asks the ROSService to subscribe to the "oscillo" topic and it listens
 * to the ROSSession's signal.
 */
void ScopeWindow::enableOscillo()
{
	if (m_script != nullptr) {
		if (m_script->rosSession() != nullptr) {
			m_script->rosSession()->callServiceOscillo("start");
			m_script->rosSession()->registerOscillo();
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
bool ScopeWindow::disableOscillo()
{
	if (m_script != nullptr) {
		if (m_script->rosSession() != nullptr) {
			return m_script->rosSession()->callServiceOscillo("stop");
		} else {
			qWarning() << "[ScopeWindow::enableOscillo] cannot enable oscillo: no ROSSession!";
		}
	} else {
		qWarning() << "[ScopeWindow::enableOscillo] ScopeWindow created without a Script!";
	}

	return false;
}

void ScopeWindow::onNewOscilloMessage(QVector<ScopeMessage> *scopeMessages)
{
	if (scopeMessages == nullptr) {
		qWarning() << "[ScopeWindow::onNewOscilloMessage] got empty message";
		return;
	}

	int n = scopeMessages->size();

	// The first items in the array should be the RT token, which indicates the start time of the
	// wave.
	qreal waveStart = scopeMessages->at(0).start();

	if (scopeMessages->at(0).uuid() != m_script->uuid())
		qWarning() << "First scope item is not the RT Token!";

	// Now that we have the start time, let's display the results
	for (int i = 1; i < n; i += 1) {
		ScopeMessage msg = scopeMessages->at(i);

		// If a Function report an earlier start time, skip it for this run (see kheops#19)
		if (msg.start() < waveStart) {
			qWarning() << "#" << i << ":" << QString::number(msg.start(), 'g', 15) << "<" << QString::number(waveStart, 'g', 15) << "(" << msg.uuid() << ")";
			continue;
		}

		// Check we have a corresponding scope item
		if (!m_scopeItems.contains(msg.uuid())) {
			// We skip the first item as it's the RT Token
			qWarning() << "UUID" << msg.uuid() << "not in map";
			continue;
		}

		// Check we have a corresponding scope label
		if (!m_scopeLabels.contains(msg.uuid())) {
			// We skip the first item as it's the RT Token
			qWarning() << "Label for UUID" << msg.uuid() << "not in map";
			continue;
		}

		ScopeItem *item = m_scopeItems[msg.uuid()];
		if (item == nullptr) {
			qWarning() << "Failed to fech scope item from map for UUID" << msg.uuid() << "!";
			continue;
		}

		// Set position and size of the scope item
		qreal startedAt = (msg.start() - waveStart) / 1e6; // in ms after start of script
		qreal x = m_xOffset + startedAt / m_period * (m_scene.width() - m_xOffset);
		qreal meanWidth = (msg.means() * 1e3) / m_period * (m_scene.width() - m_xOffset);
		qreal currentWidth = (msg.duration() * 1e3) / m_period * (m_scene.width() - m_xOffset);
		qreal minWidth = (msg.minDuration() * 1e3) / m_period * (m_scene.width() - m_xOffset);
		qreal maxWidth = (msg.maxDuration() * 1e3) / m_period * (m_scene.width() - m_xOffset);
		qreal y = m_yOffset + 2 + (i-1) * (ScopeItem::barHeight + 5);

//		item->setMaxRectWidth(maxWidth);
		item->setMeansRectWidth(meanWidth);
		item->setCurrentRectWidth(currentWidth);
//		item->setMinRectWidth(minWidth);
		item->setPos(x, y);

		// Set position of the scope label
		QGraphicsTextItem *label = m_scopeLabels[msg.uuid()];
		if (label == nullptr) {
			qWarning() << "Failed to fech scope label from map for UUID" << msg.uuid() << "!";
			continue;
		}

		label->setPos(0,
		              y);
	}

	delete scopeMessages;
}
