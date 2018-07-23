#include "papyruswindow.h"
#include "constants.h"

#include <QApplication>
#include <QWindow>
#include <QIcon>
#include <QScreen>
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Query the screen's (available) size to set the main window's size
    QScreen *screen = QGuiApplication::primaryScreen();
    if (screen == NULL) {
        qFatal("No screen detected!");
    }

    QRect availableGeometry = screen->availableGeometry();

    PapyrusWindow mainWindow(argc, argv, availableGeometry);
    mainWindow.setWindowTitle(APP_NAME);
    mainWindow.setWindowIcon(QIcon(":/icons/icons/papyrus.svg"));
    mainWindow.show();

    return app.exec();
}
