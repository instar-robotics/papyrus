#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include <iostream>

PapyrusWindow::PapyrusWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::PapyrusWindow)
{
    ui->setupUi(this);

    // Show a normal status message on application startup
    QString initialMsg = tr(APP_NAME);
    initialMsg.append(" ");
    initialMsg.append(tr("is ready"));
    ui->statusBar->showMessage(initialMsg);

    // Set initial panel sizes
    QList<int> sizes;
    sizes << 230 << 1088 << 250;
    ui->splitter->setSizes(sizes);
}

PapyrusWindow::~PapyrusWindow()
{
    delete ui;
}

/*
 * Action executed when the user wants to exit the application
 * WARNING: no saving is performed nor is the user asked about it!
 */
void PapyrusWindow::on_actionExit_triggered()
{
    //std::cout << "Exiting " << APP_NAME << " without checking for saving!" << std::endl;
    qApp->exit();
}
