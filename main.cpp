#include "papyruswindow.h"
#include "constants.h"

#include <QApplication>
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    /*
    // Parse the description directory
    QDir description(DESCRIPTION_PATH);

    // Check that the description directory exists
    if (!description.exists()) {
        QMessageBox msgBox;
        msgBox.setText(QObject::tr("<strong>Failed to load neural boxes' description files.</strong>"));

        QString str("The path ");
        str += "<em>";
        str += DESCRIPTION_PATH;
        str += "</em>";
        str += " doesn't exist. Since there is not minimal subset (as of now), the application is "
               "going to exit.";

        msgBox.setInformativeText(QObject::tr(qPrintable(str)));
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.exec();
        return -1;
    }
    //*/

    PapyrusWindow mainWindow;
    mainWindow.setWindowTitle(APP_NAME);
    mainWindow.show();

    return app.exec();
}
