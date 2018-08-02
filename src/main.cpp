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

    PapyrusWindow mainWindow(argc, argv);
    mainWindow.setWindowTitle(APP_NAME);
    mainWindow.setWindowIcon(QIcon(":/icons/icons/papyrus.svg"));
    mainWindow.show();

    return app.exec();
}
