#include "papyruswindow.h"
#include "constants.h"
#include "messagehandler.h"

#include <QApplication>
#include <QWindow>
#include <QIcon>
#include <QScreen>
#include <iostream>

int main(int argc, char *argv[])
{
	qInstallMessageHandler(coloredMessageHandler);
	QApplication app(argc, argv);

	// Set application default's font to Open Sans
	QFont globalFont = app.font();
	globalFont.setFamily("Open Sans");
	QApplication::setFont(globalFont);

	ros::init(argc, argv, "Papyrus");

	ros::NodeHandle nh; // Creating this node handles fully initializes ROS ressources

	PapyrusWindow mainWindow(argc, argv);
	mainWindow.setWindowTitle(APP_NAME);
	mainWindow.setWindowIcon(QIcon(":/icons/icons/papyrus.svg"));
	mainWindow.show();

	return app.exec();
}
