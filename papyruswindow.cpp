#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "diagramscene.h"
#include "diagramview.h"
#include "diagrambox.h"

#include <iostream>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>

PapyrusWindow::PapyrusWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::PapyrusWindow)
{
    ui->setupUi(this);
    nbPage = 1;

    // Show a normal status message on application startup
    QString initialMsg = tr(APP_NAME);
    initialMsg.append(" ");
    initialMsg.append(tr("is ready"));
    ui->statusBar->showMessage(initialMsg);

    // Set initial panel size
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
    std::cout << "Exiting " << APP_NAME << " without checking for saving!" << std::endl;
    qApp->exit();
}

void PapyrusWindow::on_btnNewScene_clicked()
{
    /*
     * ATTENTION: memory is leaking here: we should keep the pointers and delete them when we close
     * the tab. Or maybe we should look into smart pointers from the std lib.
     */

    // Create a new scene to contain the items for the new script
    DiagramScene *newScene = new DiagramScene;
    DiagramBox *newBox = new DiagramBox;
    newScene->addItem(newBox);

    // Create a new view to display the new scene
    DiagramView *newView = new DiagramView(newScene);

    // Add the new scene as a new tab
    QString str("Tab ");
    str += QString::number(nbPage);
    nbPage += 1;
    ui->tabWidget->setCurrentIndex(ui->tabWidget->addTab(newView,
                                                         QIcon(":/icons/script.svg"),
                                                         str));
}

void PapyrusWindow::on_actionAntialiasing_toggled(bool antialiasing)
{
    /*
     * ATTENTION: it only toggles the antialiasing for the current script
     * It should probably be done for all scripts
     */
    //DiagramView *currentView = qobject_cast<DiagramView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    QGraphicsView *currentView = qobject_cast<QGraphicsView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    currentView->setRenderHint(QPainter::Antialiasing, antialiasing);
    std::cout << "Antialias set to " << antialiasing << std::endl;
}
