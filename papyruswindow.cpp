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

    // Bind buttons events

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
    // We should probably store the views to prevent leaking
    //QGraphicsView *page = new QGraphicsView;
    DiagramView *page = new DiagramView;

    page->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    //page->setDragMode(QGraphicsView::RubberBandDrag);

    QString str("Script ");
    str += QString::number(nbPage);
    nbPage += 1;

    DiagramScene *scene = new DiagramScene(str, 0);
    page->setScene(scene);
    connect(scene, SIGNAL(zoomIn()), this, SLOT(zPlus()));
    connect(scene, SIGNAL(zoomOut()), this, SLOT(zMinus()));
    connect(scene, SIGNAL(selectionChanged()), this, SLOT(selectionChanged()));

    //ui->tabWidget->addTab(page, str);
    ui->tabWidget->setCurrentIndex(ui->tabWidget->addTab(page, QIcon(":/icons/script.svg"), str));
}

void PapyrusWindow::zPlus()
{
    DiagramView *currView = qobject_cast<DiagramView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    currView->scale(1.1, 1.1);
}

void PapyrusWindow::zMinus()
{
    DiagramView *currView = qobject_cast<DiagramView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    currView->scale(1 / 1.1, 1 / 1.1);
}

void PapyrusWindow::selectionChanged()
{
    /*
    DiagramView *currView = qobject_cast<DiagramView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    QGraphicsScene *currScene = currView->scene();
    //QList<QGraphicsItem *> selectedItems = currScene->selectedItems();
    QList<QGraphicsItem *> currScene->items();

    for (int i = 0; i < selectedItems.count(); i += 1) {
        DiagramBox *item = qgraphicsitem_cast<DiagramBox *>(selectedItems.at(i));
        item->setPen(QPen(Qt::black, 3));
    }
    */
}
