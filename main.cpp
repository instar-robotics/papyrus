#include "papyruswindow.h"
#include "constants.h"

#include <QApplication>
#include <QWindow>
#include <QIcon>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    PapyrusWindow mainWindow;
    mainWindow.setWindowTitle(APP_NAME);
    mainWindow.setWindowIcon(QIcon(":/icons/icons/papyrus.svg"));
    mainWindow.show();

    return app.exec();
}
