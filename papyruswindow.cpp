#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "diagramscene.h"
#include "diagramview.h"
#include "diagrambox.h"
#include "xmldescriptionreader.h"

#include <iostream>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QMessageBox>
#include <QInputDialog>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QXmlStreamReader>

PapyrusWindow::PapyrusWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::PapyrusWindow)
{
    bool libraryParsingError = false;

    ui->setupUi(this);

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

    m_library = new Library;

    description_ = description;

    // Parse the description directory
    description_.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot);
    QStringList categories = description_.entryList();

    libraryPanel_ = new LibraryPanel;

    librarySearchField_ = new QLineEdit;
    librarySearchField_->setPlaceholderText(tr("Filter..."));
    librarySearchField_->setClearButtonEnabled(true);
    librarySearchField_->setFrame(false);
    connect(librarySearchField_, SIGNAL(textChanged(QString)), this, SLOT(filterLibraryNames(QString)));

    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(librarySearchField_);
    vbox->addWidget(libraryPanel_);

    QGroupBox *libraryGroupBox = new QGroupBox(tr("Library"));
    libraryGroupBox->setLayout(vbox);

    ui->splitter->insertWidget(0, libraryGroupBox);

    // Create one 'Tree Root' per category
    for (int i = 0; i < categories.size(); i += 1) {
        // Add the category in the Tree Widget
//        QTreeWidgetItem *newCategory = addTreeRoot(categories[i]);
        Category *newCategory = addTreeRoot(categories[i]);

//        Category *cat = new Category(categories[i]);

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

            QFile xmlFile(category.absoluteFilePath(neuralBoxes[j]));
            if (!xmlFile.open(QFile::ReadOnly | QFile::Text)) {
                std::cerr << "Could not open file " << qPrintable(neuralBoxes[j]) << " for parsing" << std::endl;
                libraryParsingError = true;
                break;
            }

            // Load icon from icon path if it exists, set missing icon otherwise
            QIcon neuralIcon;
            if (f.exists())
                neuralIcon = QIcon(iconPath);
            else
                neuralIcon = QIcon(":/icons/icons/missing-icon.svg");


//            XmlDescriptionReader xmlReader(cat);
            XmlDescriptionReader xmlReader(newCategory);
            // TODO: check return value to decide whether to add in library or not
            xmlReader.read(&xmlFile, neuralIcon);

//            addTreeChild(newCategory, QIcon(neuralIcon), neuralBoxes[j]);
        }

        m_library->addCategory(newCategory);
    }

    /*
    std::cout << std::endl << "Summary of XML parsing:" << std::endl;
    foreach (Category *c, m_library->categories()) {
        std::cout << "Category: " << qPrintable(c->name()) << std::endl;

        foreach (Function *f, c->functions()) {
            std::cout << "\t Function: " << qPrintable(f->name()) << std::endl;
            std::cout << "Inputs:" << std::endl;

            foreach (InputSlot is, f->inputs()) {
                std::cout << "\t\t" << qPrintable(is.name) << "(" << is.allowMultiple << ")" << " of type " << is.type << std::endl;
            }

            std::cout << "Output:" << std::endl;
            std::cout << "\t\t" << qPrintable(f->output().name) << " of type " << f->output().type << std::endl;
        }
    }
    //*/

    // Display a warning box if some library description files could not be read
    // TODO: display a message in the system tray instead!
    // TODO: keep a list of the XML files that failed
    if (libraryParsingError)
        std::cout << "Could not open XML file" << std::endl;
        /*
        QMessageBox::warning(this, "Problem in parsing some XML description files",
                             QString("There was a problem while parsing at least one XML library description file.\n") +
                             QString("Make sure the files in the list have the correct format according to the template"));
                             */

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

    libraryPanel_->setDragEnabled(true);
//    ui->treeWidget->setDragEnabled(true);
}

PapyrusWindow::~PapyrusWindow()
{
    delete ui;
    delete trayIcon;
    delete m_library;
}

/*
 * Add a category in the Library view
 * A category contains a number of items which are neural boxes
 */
Category *PapyrusWindow::addTreeRoot(QString name)
{
//    QTreeWidgetItem *treeItem = new QTreeWidgetItem();
    Category *treeItem = new Category(name);
    libraryPanel_->insertTopLevelItem(0, treeItem);           // Make it a top-level ("category")

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
    QSizeF widgetSize = ui->tabWidget->size();
    // Set the scene's initial rect based on the widget in which it is displayed (and centered)
    newScene->setSceneRect(QRectF(- widgetSize.width() / 2,
                                  - widgetSize.height() / 2,
                                  widgetSize.width(),
                                  widgetSize.height()));

    connect(newScene, SIGNAL(displayStatusMessage(QString)), this, SLOT(displayStatusMessage(QString)));

    // Create a new view to display the new scene
    DiagramView *newView = new DiagramView(newScene);

    // Add the new scene as a new tab
    ui->tabWidget->setCurrentIndex(ui->tabWidget->addTab(newView,
                                                         QIcon(":/icons/icons/script.svg"),
                                                         newScriptName));

    ui->statusBar->showMessage("New script '" + newScriptName + "' created.");
}

Library *PapyrusWindow::getLibrary() const
{
    return m_library;
}

void PapyrusWindow::setLibrary(Library *library)
{
    m_library = library;
}

Ui::PapyrusWindow *PapyrusWindow::getUi() const
{
    return ui;
}

QLineEdit *PapyrusWindow::librarySearchField() const
{
    return librarySearchField_;
}

void PapyrusWindow::setLibrarySearchField(QLineEdit *librarySearchField)
{
    librarySearchField_ = librarySearchField;
}

void PapyrusWindow::filterLibraryNames(const QString &text)
{
    std::cout << "Should filter on '" << qPrintable(text) << "'" << std::endl;
}

void PapyrusWindow::displayStatusMessage(const QString &text)
{
    ui->statusBar->showMessage(text);
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
