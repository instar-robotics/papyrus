/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER

  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.

  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

#include "homepage.h"
#include "constants.h"

#include <QFormLayout>
#include <QFont>
#include <QDebug>
#include <QProcessEnvironment>
#include <QLineEdit>

HomePage::HomePage(QWidget *parent)
    : QFrame(parent),
      m_title(this),
      m_rosMasterStatus(this),
      m_kNodesLabel(this),
      m_kheopsNodes(this)
{
	QString title(".:| %1 v%2.%3.%4 |:.");
	m_title.setText(title.arg(APP_NAME,
	                          QString::number(MAJOR_VERSION),
	                          QString::number(MINOR_VERSION),
	                          QString::number(BUGFIX_VERSION)));
	m_title.setAlignment(Qt::AlignHCenter);
	QFont titleFont = m_title.font();
	titleFont.setPointSizeF(1.6 * titleFont.pointSize());
	titleFont.setBold(true);
	m_title.setFont(titleFont);

	QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

	// Add a field to indicate the ROS distribution
	QLineEdit *rosDistro = new QLineEdit(env.value("ROS_DISTRO", tr("<not found>")));
	rosDistro->setReadOnly(true);

	QLineEdit *rosHost = new QLineEdit(env.value("ROS_MASTER_URI", tr("<not found>")));
	rosHost->setReadOnly(true);

	m_rosMasterStatus.setText(tr("down"));

	m_kNodesLabel.setText(QString("0 kheops nodes:"));

	QFormLayout *formLayout = new QFormLayout;
	formLayout->addRow(&m_title);
	formLayout->addRow(tr("ROS distribution:"), rosDistro);
	formLayout->addRow(tr("ROS Master URI:"), rosHost);
	formLayout->addRow(tr("ROS Master status:"), &m_rosMasterStatus);
	formLayout->addRow(&m_kNodesLabel, &m_kheopsNodes);
	setLayout(formLayout);
}

void HomePage::onRosMasterChange(bool isOnline)
{
	if (isOnline)
		m_rosMasterStatus.setText("<font color='green'>up</font>");
	else
		m_rosMasterStatus.setText("<font color='red'>down</font>");
}
