#include "papyruswindow.h"
#include "constants.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    PapyrusWindow mainWindow;
    mainWindow.setWindowTitle(APP_NAME);
    mainWindow.show();

    return app.exec();
}
