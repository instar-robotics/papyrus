#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "diagramscene.h"
#include "diagramview.h"
#include "diagrambox.h"
#include "xmldescriptionreader.h"
#include "xmlscriptreader.h"

#include <iostream>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QMessageBox>
#include <QInputDialog>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include <QFileDialog>
#include <QDebug>
#include <QDockWidget>

PapyrusWindow::PapyrusWindow(QRect availableGeometry, QWidget *parent) : QMainWindow(parent),
                                                ui(new Ui::PapyrusWindow),
                                                m_activeScript(NULL),
                                                m_propertiesPanel(NULL)
{
    bool libraryParsingError = false;

    ui->setupUi(this);

    // Set the main window's geometry if given one
    if (!availableGeometry.isNull()) {
        setGeometry(0, 0, availableGeometry.width(), availableGeometry.height());
    }

    // Parse the description directory
    QDir description(QString(RESOURCE_DIR) + "descriptions");

    // Check that the description directory exists
    if (!description.exists()) {
        QMessageBox msgBox;
        msgBox.setText(QObject::tr("<strong>Failed to load neural boxes' description files.</strong>"));

        QString str("The path ");
        str += "<em>";
        str += RESOURCE_DIR;
        str += "descriptions";
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

    m_propertiesPanel = new PropertiesPanel;

    QSplitter *leftSplitter = new QSplitter(Qt::Vertical);
    leftSplitter->addWidget(libraryGroupBox);
    leftSplitter->addWidget(m_propertiesPanel);

    ui->splitter->insertWidget(0, leftSplitter);

    // Create one 'Tree Root' per category
    for (int i = 0; i < categories.size(); i += 1) {
        // Add the category in the Tree Widget
//        QTreeWidgetItem *newCategory = addTreeRoot(categories[i]);
        Category *newCategory = addTreeRoot(categories[i]);

        // Parse the corresponding directory to find the functions
        QDir category(description.canonicalPath() + "/" + categories[i]);
        category.setNameFilters(QStringList() << "*.xml"); // Match on XML files only
        QStringList neuralBoxes = category.entryList();

        // Create one entry in the category per neural box
        for (int j = 0; j < neuralBoxes.size(); j += 1) {
            // Derive icon path name from XML file name
            QString iconFilename = neuralBoxes[j];
            iconFilename.replace(".xml", ".svg");

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

            XmlDescriptionReader xmlReader(newCategory);
            QString descriptionFile = category.absoluteFilePath(neuralBoxes[j]);
            // TODO: check return value to decide whether to add in library or not
            if (!xmlReader.read(&xmlFile, neuralIcon, descriptionFile)) {
                QMessageBox::warning(this, tr("Problems while parsing the description files"),
                                     tr("There was some issues while trying to parse the XML "
                                        "description files. Only the functions with a valid XML "
                                        "file were added to the library."));
            }
        }

        m_library->addCategory(newCategory);
    }

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

    // Set initial panels size
    QList<int> sizes;
    int librarySize = 240;
    int tabWidgetSize = geometry().width() - librarySize;
    sizes << librarySize << tabWidgetSize;
    ui->splitter->setSizes(sizes);
    libraryPanel_->setDragEnabled(true);

    QList<int> leftSizes;
    int propertiesSize = 240;
    int libSize = geometry().height() - propertiesSize;
    leftSizes << libSize << propertiesSize;
    leftSplitter->setSizes(leftSizes);

    leftSplitter->setChildrenCollapsible(false);

    QSizePolicy splitterPolicy = leftSplitter->sizePolicy();
    splitterPolicy.setHorizontalPolicy(QSizePolicy::Fixed);
    leftSplitter->setSizePolicy(splitterPolicy);

    /*
    QDockWidget *minimap = new QDockWidget;
    minimap->setFloating(true);
    minimap->setGeometry(100, 100, 400, 200);
    minimap->setFeatures(QDockWidget::DockWidgetFloatable);
    minimap->setAllowedAreas(Qt::NoDockWidgetArea);
    minimap->setWindowTitle("Minimap");
    minimap->show();
    //*/

    // Make tab's height a little smaller
    ui->tabWidget->setStyleSheet("QTabBar:tab {height: 30px;}");
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
 * Check for unsaved changes before exiting
 */
void PapyrusWindow::on_actionExit_triggered()
{
    // Check if there are unsaved scripts and warn user before quitting
    bool unsavedScripts = false;
    foreach (Script *script, m_scripts) {
        if (script->modified()) {
            unsavedScripts = true;
            break;
        }
    }

    if (unsavedScripts) {
        switch (QMessageBox::question(this, tr("Save unsaved scripts?"),
                              tr("Some scripts have unsaved changes that will be lost if you exit now.\n"
                                 "Do you want to save them?"),
                                      QMessageBox::SaveAll | QMessageBox::NoAll | QMessageBox::Cancel)) {
            case QMessageBox::Cancel:
                ui->statusBar->showMessage(tr("Cancel exit."));
                break;
            case QMessageBox::SaveAll:
                // Make a pass to save all scripts
                foreach (Script *script, m_scripts) {
                    if (script->modified()) {
                        script->save();
                    }
                }

                // Make another pass to check no scripts have been ignored ('save()' should return the status
                foreach (Script *script, m_scripts) {
                    if (script->modified()) {
                        QMessageBox::warning(this, tr("There are still some unsaved scripts"),
                                             tr("Some scripts are still unsaved, abort exit."));
                        return;
                    }
                }

                qApp->exit();
                break;
            case QMessageBox::NoAll:
                ui->statusBar->showMessage(tr("Discarding unsaved script."));
                qApp->exit();
                break;
            default:
                ui->statusBar->showMessage(tr("Cancel exit."));
        }
    } else {
        qApp->exit();
    }
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
    connect(this, SIGNAL(toggleDisplayGrid(bool)), newScene, SLOT(toggleDisplayGrid(bool)));

    // Create a new view to display the new scene
    DiagramView *newView = new DiagramView(newScene);

    // Create the new script and add it to the set of scripts
    Script *newScript = new Script(newScene, newScriptName);
    connect(newScript, SIGNAL(displayStatusMessage(QString)), this, SLOT(displayStatusMessage(QString)));
//    newScene->setScript(newScript); // Unnecessary because it is done in the constructor
    addScript(newScript);

    // Add the new scene as a new tab and make it active
    ui->tabWidget->setCurrentIndex(ui->tabWidget->addTab(newView,
                                                         QIcon(":/icons/icons/script.svg"),
                                                         newScriptName));

    m_propertiesPanel->displayScriptProperties(newScript);

    // Set the script as modified
    newScript->setStatusModified(true);

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

void PapyrusWindow::on_actionDisplay_Grid_hovered()
{
    ui->statusBar->showMessage(tr("Toggle grid display."));
}

void PapyrusWindow::on_actionDisplay_Grid_toggled(bool shouldDisplay)
{
    emit(toggleDisplayGrid(shouldDisplay));
}

void PapyrusWindow::on_actionAbout_Papyrus_triggered()
{
    QString title(tr("About %1").arg(APP_NAME));
//    QString desc("<h2>.:| %1 |:.</h2>");
    QString desc("<h2>.:| ");
    desc += APP_NAME;
    desc += " v";
    desc += QString::number(MAJOR_VERSION);
    desc += ".";
    desc += QString::number(MINOR_VERSION);
    desc += " |:.</h2>";
    desc += "Graphical programming application to easily create neural networks to be run by "
            "[insert kernel application name here]<br><br>";
    desc += "<strong>Author:</strong> Nicolas SCHOEMAEKER <pre><a href='mailto:nschoe@protonmail.com'>nschoe@protonmail.com</a></pre>";
    desc += "<strong>Source:<strong> <a href='https://google.com'>[Insert github link here]</a>";

    QMessageBox::about(this, title, desc);
}

std::set<Script *> PapyrusWindow::getScripts() const
{
    return m_scripts;
}

/**
 * @brief Add a new script (if it doesn't exist) in the set of scripts
 * @param script: the script to add
 */
void PapyrusWindow::addScript(Script *script)
{
    m_scripts.insert(script);
}

Script *PapyrusWindow::activeScript() const
{
    return m_activeScript;
}

PropertiesPanel *PapyrusWindow::propertiesPanel() const
{
    return m_propertiesPanel;
}

void PapyrusWindow::setPropertiesPanel(PropertiesPanel *propertiesPanel)
{
    m_propertiesPanel = propertiesPanel;
}

void PapyrusWindow::on_actionSave_Script_triggered()
{
    // Call the 'Save' function of the current script
    if (m_activeScript == NULL) {
        ui->statusBar->showMessage(tr("No open script to save."));
        QMessageBox::warning(this, tr("No open script to save"), tr("There is no scripts opened to save!"));
        return;
    }

    m_activeScript->save();
}

void PapyrusWindow::on_actionOpen_Script_triggered()
{
    QString scriptPath = QFileDialog::getOpenFileName(NULL, tr("Open script..."),
                                 QDir::homePath(),
                                 tr("XML files (*.xml)"));

    // Make sure the user selected a file
    if (scriptPath.isEmpty()) {
        emit displayStatusMessage(tr("Abort opening script: no selected file."));
        return;
    }

    // Open the file for reading
    QFile scriptFile(scriptPath);
    if (!scriptFile.open(QIODevice::ReadOnly)) {
        emit displayStatusMessage(tr("Could not open script file."));

        QMessageBox::warning(NULL, tr("Could not open script file"),
                             tr("We failed to open the script file for reading.\nMake sure you have "
                                "the correct permissions for this file."));
        return;
    }

    // Create the scene and script objets, which will be used by the XmlReader
    DiagramScene *openScene = new DiagramScene;
    Script *openScript = new Script(openScene);

    Q_ASSERT(openScene->script() != NULL);

    // Assign the script its filepath
    openScript->setFilePath(scriptPath);

    // Parse the script XML file
    XmlScriptReader xmlReader(openScript);
    if (!xmlReader.read(&scriptFile)) {
        QString str(tr("We could not load the script, some errors happened while parsing the XML file:\n"));
        str += xmlReader.errorString();

        QMessageBox::warning(NULL, tr("Failed to load script."), str);
    } else {
        QString msg(tr("Script '") + openScript->name() + tr("' loaded."));
        ui->statusBar->showMessage(msg);

        // Create a new view to display the new scene
        DiagramView *newView = new DiagramView(openScene);

        // Connect the necessary events for the scene and the script
        connect(openScript, SIGNAL(displayStatusMessage(QString)), this, SLOT(displayStatusMessage(QString)));
        connect(this, SIGNAL(toggleDisplayGrid(bool)), openScene, SLOT(toggleDisplayGrid(bool)));

        // Add the script in the set of opened scripts
        addScript(openScript);

        /*
         * Set the scene's initial rect based on the widget in which it is displayed (and centered)
         * if it's smaller
         */
        QSizeF widgetSize = ui->tabWidget->size();
        QRectF currentSceneRect = openScene->sceneRect();
        QRectF minSceneRect(- widgetSize.width() / 2,
                            - widgetSize.height() / 2,
                            widgetSize.width(),
                            widgetSize.height());

        if (currentSceneRect.width() < minSceneRect.width()
           || currentSceneRect.height() < minSceneRect.height()) {
            openScene->setSceneRect(minSceneRect);
        }


        // Add the new scene as a new tab and make it active
        ui->tabWidget->setCurrentIndex(ui->tabWidget->addTab(newView,
                                                             QIcon(":/icons/icons/script.svg"),
                                                             openScript->name()));
    }
}

/**
 * @brief Fires when the current tab changes. Used to update the pointer to the current active
 *        script.
 * @param index: the newly active tab index
 */
void PapyrusWindow::on_tabWidget_currentChanged(int index)
{
    Q_UNUSED(index);

    // Get the view that is displayed in the tab
    DiagramView *currentView = qobject_cast<DiagramView *>(ui->tabWidget->currentWidget());
    if (currentView == NULL) {
        // TODO: _actually_ automatically report it instead of asking the user to do it.
        ui->statusBar->showMessage(tr("Error when switching tab and trying to update active script "
                                      "(this is an internal error, you should report it."));
        m_activeScript = NULL;
        return;
    }

    // Get the scene associated with the view
    DiagramScene *currentScene = qobject_cast<DiagramScene *>(currentView->scene());
    if (currentScene == NULL) {
        // TODO: _actually_ automatically report it instead of asking the user to do it.
        ui->statusBar->showMessage(tr("Error when switching tab and trying to update active script "
                                      "(this is an internal error, you should report it."));
        m_activeScript = NULL;
        return;
    }

    // Get the script associated with the scene and set it as the active script
    m_activeScript = currentScene->script();
}
