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
#include <QMessageBox>
#include <QInputDialog>

PapyrusWindow::PapyrusWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::PapyrusWindow)
{
    ui->setupUi(this);
    nbPage = 1;

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
        qApp->exit();
    }

    description_ = description;

    // Parse the description directory
    description_.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot);
    QStringList categories = description_.entryList();

    ui->treeWidget->setColumnCount(1);        // Just the function's name (an icon is added)
    ui->treeWidget->setHeaderHidden(true);    // Hide the header, we don't need it
    ui->treeWidget->setAnimated(true);
    ui->treeWidget->setIconSize(QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));
    ui->treeWidget->setIndentation(0);
    ui->treeWidget->setRootIsDecorated(true); // Try to show the little arrow (doesn't work)

    // Create one 'Tree Root' per category
    for (int i = 0; i < categories.size(); i += 1) {
        // Add the category in the Tree Widget
        QTreeWidgetItem *newCategory = addTreeRoot(categories[i]);

        // Parse the corresponding directory to find the functions
        QDir category(description.canonicalPath() + "/" + categories[i]);
        category.setNameFilters(QStringList() << "*.xml"); // Match on XML files only
        QStringList neuralBoxes = category.entryList();

        // Create one entry in the category per neural box
        for (int j = 0; j < neuralBoxes.size(); j += 1) {
            // Derive icon path name from XML file name
            QString iconFilename = neuralBoxes[j];
            iconFilename.replace(QString(".xml"), QString(".svg"));
            QString iconPath(category.absoluteFilePath(iconFilename));
            QFile f(iconPath);

            // Load icon from icon path if it exists, set missing icon otherwise
            QIcon neuralIcon;
            if (f.exists())
                neuralIcon = QIcon(iconPath);
            else
                neuralIcon = QIcon(":/icons/icons/missing-icon.svg");

            addTreeChild(newCategory, QIcon(neuralIcon), neuralBoxes[j]);
        }
    }

    // Display a system tray if it is available
    if (QSystemTrayIcon::isSystemTrayAvailable()) {
        trayIcon = new QSystemTrayIcon(this);
        trayIcon->setIcon(QIcon(":/icons/icons/papyrus.svg"));
        trayIcon->show();
    }

    // Show a normal status message on application startup
    QString initialMsg = tr(APP_NAME);
    initialMsg.append(" ");
    initialMsg.append(tr("is ready"));
    ui->statusBar->showMessage(initialMsg);

    // Set initial panel size
    QList<int> sizes;
    sizes << 230 << 1088 << 250;
    ui->splitter->setSizes(sizes);

    // Set the first tab ('Home') not closable
//    ui->tabWidget->tabBar()->setTabButton();
}

PapyrusWindow::~PapyrusWindow()
{
    delete ui;
    delete trayIcon;
}

/*
 * Add a category in the Library view
 * A category contains a number of items which are neural boxes
 */
QTreeWidgetItem *PapyrusWindow::addTreeRoot(QString name)
{
    QTreeWidgetItem *treeItem = new QTreeWidgetItem();
    ui->treeWidget->insertTopLevelItem(0, treeItem);           // Make it a top-level ("category")

    treeItem->setText(0, name);
    treeItem->setBackground(0, QBrush(Qt::lightGray));
    treeItem->setExpanded(true);
    treeItem->setTextAlignment(0, Qt::AlignCenter);
    treeItem->setFlags(treeItem->flags() & ~Qt::ItemIsSelectable); // Make categories unselectable

    return treeItem;
}

void PapyrusWindow::addTreeChild(QTreeWidgetItem *parent, QIcon icon, QString name)
{
    QTreeWidgetItem *treeItem = new QTreeWidgetItem;
    treeItem->setIcon(0, icon);
    treeItem->setText(0, name);
    treeItem->setSizeHint(0, QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));

    parent->addChild(treeItem);
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

void PapyrusWindow::on_actionAntialiasing_toggled(bool antialiasing)
{
    /*
     * ATTENTION: it only toggles the antialiasing for the current script
     * It should probably be done for all scripts
     */
    QGraphicsView *currentView = qobject_cast<QGraphicsView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    if (currentView)
        currentView->setRenderHint(QPainter::Antialiasing, antialiasing);
    else
        ui->statusBar->showMessage(tr("Cannot toggle antialiasing: no opened script!"));
}

void PapyrusWindow::on_actionZoom_In_triggered()
{
    QGraphicsView *currentView = qobject_cast<QGraphicsView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    if (currentView)
        currentView->scale(1.2 * SCALE_FACTOR, 1.2 * SCALE_FACTOR);
    else
        ui->statusBar->showMessage(tr("Cannot zoom in: no opened script!"));
}

void PapyrusWindow::on_actionZoom_Out_triggered()
{
    QGraphicsView *currentView = qobject_cast<QGraphicsView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    if (currentView)
        currentView->scale(1 / (1.2 * SCALE_FACTOR), 1 / (1.2 * SCALE_FACTOR));
    else
        ui->statusBar->showMessage(tr("Cannot zoom out: no opened script!"));
}

void PapyrusWindow::on_actionZoom_Fit_triggered()
{
    QGraphicsView *currentView = qobject_cast<QGraphicsView *>(ui->tabWidget->widget(ui->tabWidget->currentIndex()));
    if (currentView) {
        QRectF wholeScene = currentView->scene()->itemsBoundingRect();
        currentView->fitInView(wholeScene, Qt::KeepAspectRatio);
    } else {
        ui->statusBar->showMessage(tr("Cannot zoom fit: no opened script!"));
    }
}


void PapyrusWindow::on_actionNew_script_triggered()
{
    /*
     * ATTENTION: memory is leaking here: we should keep the pointers and delete them when we close
     * the tab. Or maybe we should look into smart pointers from the std lib.
     */
    bool ok;
    QString newScriptName = QInputDialog::getText(this, tr("New script name"), tr("Name:"), QLineEdit::Normal, NEW_SCRIPT_DEFAULT_NAME, &ok);

    // Don't do anything (just print status message) if user cancels the modal window
    if (!ok) {
        ui->statusBar->showMessage(tr("New script creation cancelled."));
        return;
    }

    // Make sure the script has a name
    if (newScriptName.length() == 0) {
        newScriptName = NEW_SCRIPT_DEFAULT_NAME;
        ui->statusBar->showMessage(tr("Script without a name are not allowed, setting a default name."), 2e3);
    }

    // Create a new scene to contain the items for the new script
    DiagramScene *newScene = new DiagramScene;

    // Create a new view to display the new scene
    DiagramView *newView = new DiagramView(newScene);

    // Add the new scene as a new tab
    nbPage += 1;
    ui->tabWidget->setCurrentIndex(ui->tabWidget->addTab(newView,
                                                         QIcon(":/icons/icons/script.svg"),
                                                         newScriptName));
}

void PapyrusWindow::on_actionNew_script_hovered()
{
    ui->statusBar->showMessage(tr("Create a new neural script."));
}

void PapyrusWindow::on_actionOpen_Script_hovered()
{
    ui->statusBar->showMessage(tr("Open an existing neural script."));
}

void PapyrusWindow::on_actionSave_Script_hovered()
{
    ui->statusBar->showMessage(tr("Save the current neural script."));
}

void PapyrusWindow::on_actionZoom_In_hovered()
{
    ui->statusBar->showMessage(tr("Zoom in on the current neural script."));
}

void PapyrusWindow::on_actionZoom_Out_hovered()
{
    ui->statusBar->showMessage(tr("Zoom out on the current neural script."));
}

void PapyrusWindow::on_actionZoom_Fit_hovered()
{
    ui->statusBar->showMessage(tr("Zoom to contain the entire script."));
}
