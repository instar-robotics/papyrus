#include "homepage.h"
#include "constants.h"

#include <QFormLayout>
#include <QFont>
#include <QDebug>
#include <QProcessEnvironment>
#include <QLineEdit>

HomePage::HomePage(QWidget *parent) : QFrame(parent),
                                      m_title(NULL),
                                      m_rosMasterStatus(NULL),
                                      m_kNodesLabel(NULL),
                                      m_kheopsNodes(NULL)
{
	QString title(".:| %1 v%2.%3.%4 |:.");
	m_title = new QLabel(title.arg(APP_NAME,
	                          QString::number(MAJOR_VERSION),
	                          QString::number(MINOR_VERSION),
	                          QString::number(BUGFIX_VERSION)),
	                     this);
	m_title->setAlignment(Qt::AlignHCenter);
	QFont titleFont = m_title->font();
	titleFont.setPointSizeF(1.6 * titleFont.pointSize());
	titleFont.setBold(true);
	m_title->setFont(titleFont);

	QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

	// Add a field to indicate the ROS distribution
	QLineEdit *rosDistro = new QLineEdit(env.value("ROS_DISTRO", tr("<not found>")));
	rosDistro->setReadOnly(true);

	QLineEdit *rosHost = new QLineEdit(env.value("ROS_MASTER_URI", tr("<not found>")));
	rosHost->setReadOnly(true);

	m_rosMasterStatus = new QLabel("down");

	m_kNodesLabel = new QLabel(QString("0 kheops nodes:"));
	m_kheopsNodes = new QComboBox;

	QFormLayout *formLayout = new QFormLayout;
	formLayout->addRow(m_title);
	formLayout->addRow(tr("ROS distribution:"), rosDistro);
	formLayout->addRow(tr("ROS Master URI:"), rosHost);
	formLayout->addRow(tr("ROS Master status:"), m_rosMasterStatus);
	formLayout->addRow(m_kNodesLabel, m_kheopsNodes);
	setLayout(formLayout);
}

QLabel *HomePage::rosMasterStatus() const
{
	return m_rosMasterStatus;
}

void HomePage::setRosMasterStatus(QLabel *rosMasterStatus)
{
	m_rosMasterStatus = rosMasterStatus;
}

void HomePage::onRosMasterChange(bool isOnline)
{
	if (isOnline)
		m_rosMasterStatus->setText("<font color='green'>up</font>");
	else
		m_rosMasterStatus->setText("<font color='red'>down</font>");
}
