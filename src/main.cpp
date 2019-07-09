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

#include "papyruswindow.h"
#include "constants.h"
#include "messagehandler.h"
#include "scopemessage.h"
#include "helpers.h"

#include <QApplication>
#include <QWindow>
#include <QIcon>
#include <QScreen>
#include <iostream>
#include <QSysInfo>

int main(int argc, char *argv[])
{
	qInstallMessageHandler(coloredMessageHandler);
	QApplication app(argc, argv);

	// Register custom types
	qRegisterMetaType<ScopeMessage>();

	// Set application default's font to Open Sans
	QFont globalFont = app.font();
	globalFont.setFamily("Open Sans");
	QApplication::setFont(globalFont);

	ros::init(argc, argv, QString("Papyrus_%1")
	          .arg(sanitizeTopicName(QSysInfo::machineHostName())).toStdString());

	ros::NodeHandle nh; // Creating this node handles fully initializes ROS ressources

	PapyrusWindow mainWindow(argc, argv);
	mainWindow.setWindowTitle(APP_NAME);
	mainWindow.setWindowIcon(QIcon(":/icons/icons/papyrus.svg"));
	mainWindow.show();

	return app.exec();
}
