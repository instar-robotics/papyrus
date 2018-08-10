﻿#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "diagramscene.h"
#include "diagramview.h"
#include "diagrambox.h"
#include "xmlscriptreader.h"
#include "helpers.h"
#include "nodeschooser.h"
#include "constantfunction.h"

#include <cryptopp/filters.h>
#include <cryptopp/aes.h>
#include <cryptopp/modes.h>
#include <cryptopp/osrng.h>
#include <cryptopp/hex.h>
#include <cryptopp/files.h>

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
#include <QDialog>
#include <QFileInfo>
#include <QSizePolicy>
#include <QSettings>
#include <QScreen>
#include <QActionGroup>

PapyrusWindow::PapyrusWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::PapyrusWindow),
    m_argc(argc),
    m_argv(argv),
    m_rosMasterStatus(NULL),
    librarySearchField_(NULL),
    m_lastExpandedCategory("Constants"),
    m_libraryParsingErrors(0),
    m_activeScript(NULL),
    m_propertiesPanel(NULL),
    m_homePage(NULL),
    m_runTimeDisplay(NULL),
    m_actionDebug(NULL),
    m_actionRelease(NULL)
{
    // First of all set the UI according to the UI file (MUST be called before the rest)
    m_ui->setupUi(this);

    // Add the actions that can't be added via the qtcreator
    QActionGroup *devTypeGroup = new QActionGroup(this);
    m_actionRelease = new QAction("Release", devTypeGroup);
    m_actionRelease->setCheckable(true);
    m_actionDebug = new QAction("Debug", devTypeGroup);
    m_actionDebug->setCheckable(true);

    QAction *editPath = m_ui->actionEdit_paths;
    m_ui->menuDevelopment->insertSeparator(editPath)->setText(tr("Type"));
    m_ui->menuDevelopment->insertAction(editPath, m_actionRelease);
    m_ui->menuDevelopment->insertAction(editPath, m_actionDebug);
    m_ui->menuDevelopment->insertSeparator(editPath);

    connect(devTypeGroup, SIGNAL(triggered(QAction*)), this, SLOT(updateDevelopmentEnvironment(QAction*)));

    // Then read the QSettings (make sure to call AFTER the above, because we need these actions)
    readSettings();

    // Before trying to parse the function descriptions, check if we have the path to it, otherwise
    // ask it

    QString searchPath;

    if (m_developmentType == RELEASE) {
        if (m_releasePath.isEmpty())
            askForPath(true, PATH_DESC);

        searchPath = m_releasePath;
    } else if (m_developmentType == DEBUG) {
        if (m_debugPath.isEmpty())
            askForPath(true, PATH_DESC);

        searchPath = m_debugPath;
    } else {
        informUserAndCrash(tr("Unsupported development type."),
                           tr("Supported development types are either \"DEBUG\" or \"RELEASE\"."
                              "The current specified development type is not. This is probably due "
                              "to an API change that was not implemented."));
    }

    // Temporary create those here, because I have made the parsing dependent on this (which is stupid)
    libraryPanel_ = new LibraryPanel;

    connect(libraryPanel_, SIGNAL(itemExpanded(QTreeWidgetItem *)), this, SLOT(categoryExpanded(QTreeWidgetItem*)));

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

    m_ui->splitter->insertWidget(0, leftSplitter);

    QDir description(searchPath);

    // Make a last check in order to see if the user simply cancelled
    if (searchPath.isEmpty()) {
        QMessageBox::warning(this, tr("Library path not specified"),
        tr("You did not specify a path for the library files.\nIt is needed as ")
        + QString(APP_NAME) + tr(" needs to know where to search for those files.\nThe application "
        "will still load, but you won't be able to create scripts."));
    } else if (!description.exists()) {
        QMessageBox msgBox;
        msgBox.setText(QObject::tr("<strong>Failed to load neural boxes' description files.</strong>"));

        QString str("The path ");
        str += "<em>";
        str += description.absolutePath();
        str += "</em>";
        str += " doesn't exist, so function description files could not be loaded.\nThe application"
               " will still load, but you won't be able to create scripts.";

        msgBox.setInformativeText(QObject::tr(qPrintable(str)));
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.exec();
    }
    // Parse the description directory
    else {
        m_library = new Library;

        description_ = description;

        description_.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot);
        QStringList categories = description_.entryList();

        /*
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

        m_ui->splitter->insertWidget(0, leftSplitter);
        */

        // Create one 'Tree Root' per category
        for (int i = 0; i < categories.size(); i += 1) {
            // Add the category in the Tree Widget
            Category *newCategory = addTreeRoot(snakeCaseToPretty(categories[i]));\
            XmlDescriptionReader *xmlReader = new XmlDescriptionReader(newCategory);

            parseOneLevel(QDir(description.canonicalPath() + "/" + categories[i]), xmlReader);

            m_library->addCategory(newCategory);
        }

        // Create one "built-in" category for the constant inputs (created at the end so that it
        // appears first)
        Category *constants = addTreeRoot("Constants");
        constants->setExpanded(true); // By default, keep "Constants" expanded
        ConstantFunction *constantScalar = new ConstantFunction("SCALAR",
                                                                ":/icons/icons/constant-scalar.svg",
                                                                QIcon(":/icons/icons/constant-scalar.svg"),
                                                                SCALAR);
        ConstantFunction *constantString = new ConstantFunction("STRING",
                                                                ":icons/icons/constant-string.svg",
                                                                QIcon(":/icons/icons/constant-string.svg"),
                                                                STRING);
        ConstantFunction *constantMatrix = new ConstantFunction("MATRIX",
                                                                ":/icons/icons/constant-matrix.svg",
                                                                QIcon(":/icons/icons/constant-matrix.svg"),
                                                                MATRIX);
        constants->addChild(constantScalar);
        constants->addChild(constantString);
        constants->addChild(constantMatrix);

        // Display a warning box if some library description files could not be read
        // TODO: display a message in the system tray instead!
        // TODO: keep a list of the XML files that failed
        if (m_libraryParsingErrors > 0)
            QMessageBox::warning(this, tr("Problems while parsing the description files"),
            QString::number(m_libraryParsingErrors) + tr(" issues happened where parsing XML"
            " description files.\nOnly the functions with a valid XML "
            "file were added to the library.\nPlease fix the syntax errors in order to have the full"
            " library of functions loaded."));
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
    m_ui->statusBar->showMessage(initialMsg);

    // Add an icon to display the status of the ROS master
    QIcon rosMasterIcon(":/icons/icons/ros-master-off.svg");
    m_rosMasterStatus = new QLabel;
    m_rosMasterStatus->setPixmap(rosMasterIcon.pixmap(QSize(30, 30)));
    m_ui->statusBar->addPermanentWidget(m_rosMasterStatus);

    // Set initial panels size
    QList<int> sizes;
    int librarySize = 240;
    int tabWidgetSize = geometry().width() - librarySize;
    sizes << librarySize << tabWidgetSize;
    m_ui->splitter->setSizes(sizes);
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
    m_ui->tabWidget->setStyleSheet("QTabBar:tab {height: 30px;}");
    m_homePage = new HomePage;
    m_ui->tabWidget->addTab(m_homePage, QIcon(":/icons/icons/home.svg"), "Home");

    // Create & initialize the RosNode
    spawnRosNode();

    // Add a line edit showing the script's runtime (cannot be done from QtDesigner)
    m_runTimeDisplay = new QLineEdit("00:00:00:00");
    m_runTimeDisplay->setReadOnly(true);
    m_runTimeDisplay->setEnabled(false);
    m_runTimeDisplay->setMaximumWidth(160);
    m_runTimeDisplay->setAlignment(Qt::AlignCenter);
    m_runTimeDisplay->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    m_ui->mainToolBar->insertWidget(m_ui->actionStop, m_runTimeDisplay);

    // Create the empty ROS Session and bind to its events
    /*
    m_rosSession = new ROSSession;
    connect(m_rosSession, SIGNAL(displayStatusMessage(QString,MessageUrgency)),this, SLOT(displayStatusMessage(QString,MessageUrgency)));
    connect(m_rosSession, SIGNAL(scriptResumed()), this, SLOT(onScriptResumed()));
    connect(m_rosSession, SIGNAL(scriptPaused()), this, SLOT(onScriptPaused()));
    connect(m_rosSession, SIGNAL(scriptStopped()), this, SLOT(onScriptStopped()));
    connect(m_rosSession, SIGNAL(timeElapsed(int,int,int,int)), this, SLOT(updateStopWatch(int,int,int,int)));
    */
}

PapyrusWindow::~PapyrusWindow()
{
    delete m_ui;
    delete trayIcon;
    delete m_library;
    delete m_rosnode;
    delete m_rosMasterStatus;
}

/**
 * @brief PapyrusWindow::closeEvent is the event received when asked to close the main window.
 * We use it to save the application settings
 * @param evt
 */
void PapyrusWindow::closeEvent(QCloseEvent *evt)
{
    writeSettings();
    evt->accept();
}

/**
 * @brief PapyrusWindow::readSettings reads the application settings and apply them (restore state, preferences, etc.)
 */
void PapyrusWindow::readSettings()
{
    QSettings settings(ORGA, APP_NAME);

    // Query the screen's (available) size to set the main window's size
    QScreen *screen = QGuiApplication::primaryScreen();
    if (screen == NULL) {
        qFatal("No screen detected!");
    }
    QSize availableSize = screen->availableSize();

    // Restore the state of the main window
    settings.beginGroup("MainWindow");
    resize(settings.value("size", availableSize).toSize());
    move(settings.value("pos", QPoint(33, 33)).toPoint());
    settings.endGroup(); // MainWindow

    // View settings regarding the scripts (grid toggled, antialiasing, etc.)
    settings.beginGroup("View");
    m_ui->actionAntialiasing->setChecked(settings.value("antialiasing", true).toBool());
    m_ui->actionDisplay_Grid->setChecked(settings.value("displayGrid", true).toBool());
    settings.endGroup(); // View

    // Development environment settings
    settings.beginGroup("Development");
    m_developmentType = settings.value("type", RELEASE).value<DevelopmentType>();
    m_actionDebug->setChecked(m_developmentType == DEBUG);
    m_actionRelease->setChecked(m_developmentType == RELEASE);
    m_debugPath = settings.value("debugPath", "").toString();
    m_releasePath = settings.value("releasePath", "").toString();
    m_debugLibPath = settings.value("debugLibPath", "").toString();
    m_releaseLibPath = settings.value("releaseLibPath", "").toString();
    settings.endGroup(); // Development

    // Cipher sections for crypting / decrypting script files
    settings.beginGroup("Cipher");
    m_keyFile = settings.value("keyFile", "").toString();
    m_ivFile = settings.value("ivFile", "").toString();
    settings.endGroup(); // Cipher
}

/**
 * @brief PapyrusWindow::writeSettings is called when the main window is closed, and saves all
 * settings, to be re-loaded next time the application is launched
 */
void PapyrusWindow::writeSettings()
{
    QSettings settings(ORGA, APP_NAME);

    // Save the main window's state
    settings.beginGroup("MainWindow");
    settings.setValue("size", size());
    settings.setValue("pos", pos());
    settings.endGroup(); // MainWindow

    // Save the view settings
    settings.beginGroup("View");
    settings.setValue("antialiasing", m_ui->actionAntialiasing->isChecked());
    settings.setValue("displayGrid", m_ui->actionDisplay_Grid->isChecked());
    settings.endGroup(); // View

    // Development environment settings
    settings.beginGroup("Development");
    settings.setValue("type", m_developmentType);
    settings.setValue("debugPath", m_debugPath);
    settings.setValue("releasePath", m_releasePath);
    settings.setValue("debugLibPath", m_debugLibPath);
    settings.setValue("releaseLibPath", m_releaseLibPath);
    settings.endGroup(); // Development

    // Cipher section to crypt / decrypt script files
    settings.beginGroup("Cipher");
    settings.setValue("keyFile", m_keyFile);
    settings.setValue("ivFile", m_ivFile);
    settings.endGroup(); // Cipher
}

/*
 * Add a category in the Library view
 * A category contains a number of items which are neural boxes
 */
Category *PapyrusWindow::addTreeRoot(QString name)
{
    Category *treeItem = new Category(name);
    libraryPanel_->insertTopLevelItem(0, treeItem);           // Make it a top-level ("category")

    treeItem->setText(0, name);
//    treeItem->setBackground(0, QBrush(Qt::lightGray));
    treeItem->setBackground(0, QBrush(QColor(0xc7ebff)));
    treeItem->setTextAlignment(0, Qt::AlignCenter);
    treeItem->setFlags(treeItem->flags() & ~Qt::ItemIsSelectable); // Make categories unselectable

    return treeItem;
}

/*
void PapyrusWindow::addTreeChild(QTreeWidgetItem *parent, const QIcon &icon, const QString &name)
{
    QTreeWidgetItem *treeItem = new QTreeWidgetItem;
    treeItem->setIcon(0, icon);
    treeItem->setText(0, name);
    treeItem->setSizeHint(0, QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));

    parent->addChild(treeItem);
}
*/
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
                m_ui->statusBar->showMessage(tr("Cancel exit."));
                break;
            case QMessageBox::SaveAll:
                // Make a pass to save all scripts
                foreach (Script *script, m_scripts) {
                    if (script->modified()) {
                        script->save(getDescriptionPath());
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

                close();
                break;
            case QMessageBox::NoAll:
                m_ui->statusBar->showMessage(tr("Discarding unsaved script."));
                close();
                break;
            default:
                m_ui->statusBar->showMessage(tr("Cancel exit."));
        }
    } else {
        // Closing the main window will make the application exit
        close();
    }
}

void PapyrusWindow::on_actionAntialiasing_toggled(bool antialiasing)
{
    /*
     * ATTENTION: it only toggles the antialiasing for the current script
     * It should probably be done for all scripts
     */
    QGraphicsView *currentView = dynamic_cast<QGraphicsView *>(m_ui->tabWidget->widget(m_ui->tabWidget->currentIndex()));
    if (currentView)
        currentView->setRenderHint(QPainter::Antialiasing, antialiasing);
    else
        m_ui->statusBar->showMessage(tr("Cannot toggle antialiasing: no opened script!"));
}

void PapyrusWindow::on_actionZoom_In_triggered()
{
    QGraphicsView *currentView = dynamic_cast<QGraphicsView *>(m_ui->tabWidget->widget(m_ui->tabWidget->currentIndex()));
    if (currentView)
        currentView->scale(1.2 * SCALE_FACTOR, 1.2 * SCALE_FACTOR);
    else
        m_ui->statusBar->showMessage(tr("Cannot zoom in: no opened script!"));
}

void PapyrusWindow::on_actionZoom_Out_triggered()
{
    QGraphicsView *currentView = dynamic_cast<QGraphicsView *>(m_ui->tabWidget->widget(m_ui->tabWidget->currentIndex()));
    if (currentView)
        currentView->scale(1 / (1.2 * SCALE_FACTOR), 1 / (1.2 * SCALE_FACTOR));
    else
        m_ui->statusBar->showMessage(tr("Cannot zoom out: no opened script!"));
}

void PapyrusWindow::on_actionZoom_Fit_triggered()
{
    QGraphicsView *currentView = dynamic_cast<QGraphicsView *>(m_ui->tabWidget->widget(m_ui->tabWidget->currentIndex()));
    if (currentView) {
        QRectF wholeScene = currentView->scene()->itemsBoundingRect();
        currentView->fitInView(wholeScene, Qt::KeepAspectRatio);
    } else {
        m_ui->statusBar->showMessage(tr("Cannot zoom fit: no opened script!"));
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
        m_ui->statusBar->showMessage(tr("New script creation cancelled."));
        return;
    }

    // Make sure the script has a name
    if (newScriptName.length() == 0) {
        newScriptName = NEW_SCRIPT_DEFAULT_NAME;
        m_ui->statusBar->showMessage(tr("Script without a name are not allowed, setting a default name."), 2e3);
    }

    // Create a new scene to contain the items for the new script
    DiagramScene *newScene = new DiagramScene;
    QSizeF widgetSize = m_ui->tabWidget->size();
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
    connect(newScript, SIGNAL(displayStatusMessage(QString,MessageUrgency)), this,
            SLOT(displayStatusMessage(QString,MessageUrgency)));
    connect(newScript, SIGNAL(scriptPaused()), this, SLOT(onScriptPaused()));
    connect(newScript, SIGNAL(scriptResumed()), this, SLOT(onScriptResumed()));
    connect(newScript, SIGNAL(scriptStopped()), this, SLOT(onScriptStopped()));
//    connect(newScript, SIGNAL(timeElapsed(int,int,int,int)), this,
//            SLOT(updateStopWatch(int,int,int,int)));
    addScript(newScript);

    // Add the new scene as a new tab and make it active
    m_ui->tabWidget->setCurrentIndex(m_ui->tabWidget->addTab(newView,
                                                         QIcon(":/icons/icons/script.svg"),
                                                         newScriptName));

    m_propertiesPanel->displayScriptProperties(newScript);

    // Set the script as modified
    newScript->setStatusModified(true);

    m_ui->statusBar->showMessage("New script '" + newScriptName + "' created.");
}

Library *PapyrusWindow::getLibrary() const
{
    return m_library;
}

void PapyrusWindow::setLibrary(Library *library)
{
    m_library = library;
}

Ui::PapyrusWindow *PapyrusWindow::ui() const
{
    return m_ui;
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
    bool exp = !text.isEmpty();

    int n = libraryPanel_->topLevelItemCount();
    for (int i = 0; i < n; i += 1) {
        libraryPanel_->topLevelItem(i)->setExpanded(exp);
    }

    for (int i = 0; i < n; i += 1) {
        Category *cat = dynamic_cast<Category *>(libraryPanel_->topLevelItem(i));
        if (cat == NULL) {
            qDebug() << "Failed to cast Cat";
            continue;
        }

        int m = cat->childCount();
        for (int j = 0; j < m; j += 1) {
            Function *f = dynamic_cast<Function *>(cat->child(j));
            if (f == NULL) {
                qDebug() << "Failed to cast Func";
                continue;
            }

            // If we have some text to filter, check the matching
            if (exp) {
                if (f->name().toLower().contains(text.toLower())) {
                    f->setHidden(false);
                } else {
                    f->setHidden(true);
                }
            }
            // Otherwise, restore all hidden states
            else {
                f->setHidden(false);
            }
        }

        // If we don't have text to filter, expand back the last category (as well as Constants)
        if (!exp && (cat->name() == m_lastExpandedCategory || cat->name() == "Constants")) {
            cat->setExpanded(true);
        }
    }

}

void PapyrusWindow::displayStatusMessage(const QString &text, MessageUrgency urgency)
{
    switch (urgency) {
    case MSG_INFO:
        m_ui->statusBar->setStyleSheet("color: black");
        break;

    case MSG_WARNING:
        m_ui->statusBar->setStyleSheet("color: orange");
        break;

    case MSG_ERROR:
        m_ui->statusBar->setStyleSheet("color: red");
        break;
    }

    m_ui->statusBar->showMessage(text);
}

/**
 * @brief PapyrusWindow::onROSMasterChange changed the status' bar ROS icon to reflect when the ROS
 * master goes ON and OFF
 * @param isOnline the new status of the ROS master
 */
void PapyrusWindow::onROSMasterChange(bool isOnline)
{
    if (isOnline) {
        QIcon rosMasterIconON(":/icons/icons/ros-master-on.svg");
        m_rosMasterStatus->setPixmap(rosMasterIconON.pixmap(QSize(30, 30)));
        m_ui->statusBar->showMessage(tr("The ROS master just went online"));

        trayIcon->showMessage(tr("ROS Master just went back up!"),
                                      tr("The ROS Master just went back online!\nSo connections with "
                                         "services and topics should be available again."),
                                      QSystemTrayIcon::Information);
    } else {
        QIcon rosMasterIconOFF(":/icons/icons/ros-master-off.svg");
        m_rosMasterStatus->setPixmap(rosMasterIconOFF.pixmap(QSize(30, 30)));
        m_ui->statusBar->showMessage(tr("The ROS master just went offline"));

        trayIcon->showMessage(tr("ROS Master just went down"),
                              tr("The ROS Master just went offline, so connection with every ROS "
                                 "topics or services are currently unavailable!"),
                              QSystemTrayIcon::Warning);

        // Stop the current thread, and recreate one (to re-init ROS, etc.)
        delete m_rosnode;
        spawnRosNode();
    }
}

/**
 * @brief PapyrusWindow::onScriptResumed is called when the @ROSSession confirms the script was
 * resumed
 */
void PapyrusWindow::onScriptResumed()
{
    // Make the play/pause button into its "pause" configuration
    m_ui->actionRun->setIcon(QIcon(":/icons/icons/pause.svg"));
    m_ui->actionRun->setToolTip(tr("Pause script"));
    m_ui->actionRun->setEnabled(true);

    // Enable the stop button
    m_ui->actionStop->setEnabled(true);

    // Enable the scope button
    m_ui->actionScope->setEnabled(true);

    // Display a message in the status bar
    displayStatusMessage(tr("Script \"%1\" resumed").arg(m_activeScript->rosSession()->nodeName()));
}

/**
 * @brief PapyrusWindow::onScriptPaused is called when the @ROSSession confirms the script was
 * paused
 */
void PapyrusWindow::onScriptPaused()
{
    // Make the play/pause button into its "play" configuration
    m_ui->actionRun->setIcon(QIcon(":/icons/icons/play.svg"));
    m_ui->actionRun->setToolTip(tr("Resume script"));
    m_ui->actionRun->setEnabled(true);

    // Enable the stop button
    m_ui->actionStop->setEnabled(true);

    // Enable the scope button
    // TODO: does it make sense to have scope when paused ?
    m_ui->actionScope->setEnabled(true);

    // Display a message in the status bar
    displayStatusMessage(tr("Script \"%1\" paused").arg(m_activeScript->rosSession()->nodeName()));
}

/**
 * @brief PapyrusWindow::onScriptStopped is called when the @ROSSession confirms the script was
 * stopped
 */
void PapyrusWindow::onScriptStopped()
{
    // Make the play/pause button into its "play" configuration
    m_ui->actionRun->setIcon(QIcon(":/icons/icons/play.svg"));
    m_ui->actionRun->setToolTip(tr("Start script"));             // Note here "start" vs "resume" :)
    m_ui->actionRun->setEnabled(true);

    // Disable the stop button
    m_ui->actionStop->setEnabled(false);

    // disable scope button
    m_ui->actionScope->setEnabled(false);

    // Reset stopwatch
    updateStopWatch(0, 0, 0, 0);

    // Display a message in the status bar
    displayStatusMessage(tr("Script \"%1\" stopped").arg(m_activeScript->rosSession()->nodeName()));
}

void PapyrusWindow::updateStopWatch(int h, int m, int s, int ms)
{
//    m_runTimeDisplay->setText(QString("%1:%2:%3:%4").arg(QString::number(h), QString::number(m), QString::number(s), QString::number(ms)));
    QChar pad = QChar('0');
    int hundreths = ms / 10; // We don't want to display ms precision, only hundredth
    m_runTimeDisplay->setText(QString("%1:%2:%3:%4")
                              .arg(h, 2, 10, pad)
                              .arg(m, 2, 10, pad)
                              .arg(s, 2, 10, pad)
                              .arg(hundreths, 2, 10, pad));
}

/**
 * @brief PapyrusWindow::updateDevelopmentEnvironment is used to toggle the development environment
 * when selected from the menu
 * @param action
 */
void PapyrusWindow::updateDevelopmentEnvironment(QAction *action)
{
    if (action == m_actionDebug)
        m_developmentType = DEBUG;
    else if (action == m_actionRelease)
        m_developmentType = RELEASE;
}

void PapyrusWindow::on_actionNew_script_hovered()
{
    m_ui->statusBar->showMessage(tr("Create a new neural script."));
}

void PapyrusWindow::on_actionOpen_Script_hovered()
{
    m_ui->statusBar->showMessage(tr("Open an existing neural script."));
}

void PapyrusWindow::on_actionSave_Script_hovered()
{
    m_ui->statusBar->showMessage(tr("Save the current neural script."));
}

void PapyrusWindow::on_actionZoom_In_hovered()
{
    m_ui->statusBar->showMessage(tr("Zoom in on the current neural script."));
}

void PapyrusWindow::on_actionZoom_Out_hovered()
{
    m_ui->statusBar->showMessage(tr("Zoom out on the current neural script."));
}

void PapyrusWindow::on_actionZoom_Fit_hovered()
{
    m_ui->statusBar->showMessage(tr("Zoom to contain the entire script."));
}

void PapyrusWindow::on_actionDisplay_Grid_hovered()
{
    m_ui->statusBar->showMessage(tr("Toggle grid display."));
}

void PapyrusWindow::on_actionDisplay_Grid_toggled(bool shouldDisplay)
{
    emit(toggleDisplayGrid(shouldDisplay));
}

void PapyrusWindow::on_actionAbout_Papyrus_triggered()
{
    QString title(tr("About %1").arg(APP_NAME));
    QString desc("<h2>.:| ");
    desc += APP_NAME;
    desc += " v";
    desc += QString::number(MAJOR_VERSION);
    desc += ".";
    desc += QString::number(MINOR_VERSION);
    desc += " |:.</h2>";
    desc += "Graphical programming application to easily create neural networks to be run by "
            "kheops<br><br>";
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

QSystemTrayIcon *PapyrusWindow::getTrayIcon() const
{
    return trayIcon;
}

RosNode *PapyrusWindow::rosnode() const
{
    return m_rosnode;
}

void PapyrusWindow::setRosnode(RosNode *rosnode)
{
    m_rosnode = rosnode;
}

void PapyrusWindow::spawnRosNode()
{
    m_rosnode = new RosNode(m_argc, m_argv);
    connect(m_rosnode, SIGNAL(rosMasterChanged(bool)), this, SLOT(onROSMasterChange(bool)));
    connect(m_rosnode, SIGNAL(rosMasterChanged(bool)), m_homePage, SLOT(onRosMasterChange(bool)));
    m_rosnode->init();
}

HomePage *PapyrusWindow::homePage() const
{
    return m_homePage;
}

void PapyrusWindow::setHomePage(HomePage *homePage)
{
    m_homePage = homePage;
}

DevelopmentType PapyrusWindow::developmentType() const
{
    return m_developmentType;
}

QString PapyrusWindow::debugPath() const
{
    return m_debugPath;
}

QString PapyrusWindow::releasePath() const
{
    return m_releasePath;
}

QString PapyrusWindow::debugLibPath() const
{
    return m_debugLibPath;
}

QString PapyrusWindow::releaseLibPath() const
{
    return m_releaseLibPath;
}

QString PapyrusWindow::keyFile() const
{
    return m_keyFile;
}

void PapyrusWindow::setKeyFile(const QString &keyFile)
{
    m_keyFile = keyFile;
}

QString PapyrusWindow::ivFile() const
{
    return m_ivFile;
}

void PapyrusWindow::setIvFile(const QString &ivFile)
{
    m_ivFile = ivFile;
}

void PapyrusWindow::on_actionSave_Script_triggered()
{
    // Call the 'Save' function of the current script
    if (m_activeScript == NULL) {
        m_ui->statusBar->showMessage(tr("No open script to save."));
        QMessageBox::warning(this, tr("No open script to save"), tr("There is no scripts opened to save!"));
        return;
    }

    m_activeScript->save(getDescriptionPath());
}

void PapyrusWindow::on_actionOpen_Script_triggered()
{
    QString scriptPath = QFileDialog::getOpenFileName(NULL, tr("Open script..."),
                                 QDir::homePath(),
                                 tr("XML files (*.xml);; Crypted XML files (*.xml.crypted)"));

    // Make sure the user selected a file
    if (scriptPath.isEmpty()) {
        emit displayStatusMessage(tr("Abort opening script: no selected file."));
        return;
    }

    // Check if the file is an encrypted file, and if yes, decrypt it
    QFileInfo fi(scriptPath);
    if (fi.completeSuffix() == "xml.crypted") {
        // Check that the user has filled in a key and iv path
        if (m_keyFile.isEmpty() || m_ivFile.isEmpty()) {
            QMessageBox::warning(NULL, tr("Missing crypto information"),
                                 tr("You are trying to decrypt an encrypted script file, for "
                                    "this, we need a decryption key and IV.\nWe "
                                    "detected that at least one is missing. You will be prompted for"
                                    " paths in the next window, read the dialog's titles in order to"
                                    " provide either the key or the IV (don't get them mixed!)."));
        }

        if (m_keyFile.isEmpty()) {
            m_keyFile = QFileDialog::getOpenFileName(this, tr("Please provide the KEY file"),
                                                     tr("/home"),
                                                     "Key files (*)");
        }

        if (m_ivFile.isEmpty()) {
            m_ivFile = QFileDialog::getOpenFileName(this, tr("Please provide the IV file"),
                                                    tr("/home"),
                                                    "IV files (*)");
        }

        // Make another check to be sure the user did not cancel
        if (m_keyFile.isEmpty() || m_ivFile.isEmpty()) {
            QMessageBox::warning(NULL, tr("Loading aborted"),
                                 tr("The script was not loaded because you failed to provide either "
                                    "the key or the IV file for decryption."));
            return;
        }

        // Check that we can read the key and IV
        if (!fileExists(m_keyFile.toStdString()) || !fileExists(m_ivFile.toStdString())) {
            QMessageBox::warning(NULL, tr("Encryption key and IV not found"),
                                 tr("We could not open this encrypted script file because either the"
                                    " key of the IV file could not be found."));
            return;
        }

        // Read and store key
        std::string key;
        CryptoPP::FileSource fkey(m_keyFile.toStdString().c_str(), true,
                                  new CryptoPP::HexDecoder(
                                      new CryptoPP::StringSink(key)));

        // Read and store iv
        std::string iv;
        CryptoPP::FileSource fiv(m_ivFile.toStdString().c_str(), true,
                                 new CryptoPP::HexDecoder(
                                     new CryptoPP::StringSink(iv)));

        Q_UNUSED(fkey);
        Q_UNUSED(fiv);

        // Create decryptor
        CryptoPP::CBC_Mode<CryptoPP::AES>::Decryption dec;
        dec.SetKeyWithIV(reinterpret_cast<const byte *>(key.data()),
                         key.size(),
                         reinterpret_cast<const byte *>(iv.data()));

        QString tmpDestFile(scriptPath);
        tmpDestFile.replace(".crypted", ".decrypted");

        // Decrypt the file
        CryptoPP::FileSource f(scriptPath.toStdString().c_str(), true,
                               new CryptoPP::StreamTransformationFilter(dec,
                                   new CryptoPP::FileSink(tmpDestFile.toStdString().c_str())));

        Script *newScript = parseXmlScriptFile(tmpDestFile);

        if (newScript != NULL) {
            // Remove the temporary clear file used to decrypt
            QFile::remove(tmpDestFile);

            // Assign the script its new filepath (remove ".decrypted" extension)
            newScript->setFilePath(tmpDestFile.replace(".decrypted", ""));

            // Also set the script as encrypted
            newScript->setEncrypt(true);
        }
    } else {
        parseXmlScriptFile(scriptPath);
    }
}

Script *PapyrusWindow::parseXmlScriptFile(const QString &scriptPath)
{
    // Open the file for reading
    QFile scriptFile(scriptPath);
    if (!scriptFile.open(QIODevice::ReadOnly)) {
        emit displayStatusMessage(tr("Could not open script file."));

        QMessageBox::warning(NULL, tr("Could not open script file"),
                             tr("We failed to open the script file for reading.\nMake sure you have "
                                "the correct permissions for this file."));
        return NULL;
    }

    // Create the scene and script objets, which will be used by the XmlReader
    DiagramScene *openScene = new DiagramScene;
    Script *openScript = new Script(openScene);
    connect(openScript, SIGNAL(displayStatusMessage(QString,MessageUrgency)), this,
            SLOT(displayStatusMessage(QString,MessageUrgency)));
    connect(openScript, SIGNAL(scriptPaused()), this, SLOT(onScriptPaused()));
    connect(openScript, SIGNAL(scriptResumed()), this, SLOT(onScriptResumed()));
    connect(openScript, SIGNAL(scriptStopped()), this, SLOT(onScriptStopped()));
    //    connect(openScript, SIGNAL(timeElapsed(int,int,int,int)), this,
    //            SLOT(updateStopWatch(int,int,int,int)));

    Q_ASSERT(openScene->script() != NULL);

    openScript->setFilePath(scriptPath);

    // Parse the script XML file
    XmlScriptReader xmlReader(openScript, getDescriptionPath());
    if (!xmlReader.read(&scriptFile)) {
        QString str(tr("We could not load the script, some errors happened while parsing the XML file:\n"));
        str += xmlReader.errorString();

        QMessageBox::warning(NULL, tr("Failed to load script."), str);

        return NULL;
    } else {
        QString msg(tr("Script '") + openScript->name() + tr("' loaded."));
        m_ui->statusBar->showMessage(msg);

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
        QSizeF widgetSize = m_ui->tabWidget->size();
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
        m_ui->tabWidget->setCurrentIndex(m_ui->tabWidget->addTab(newView,
                                                             QIcon(":/icons/icons/script.svg"),
                                                             openScript->name()));
    }

    return openScript;
}

/**
 * @brief PapyrusWindow::askLibraryPath fires a modal window used to ask the user for the path
 * of the library files (either in debug of release mode)
 * @return
 */
void PapyrusWindow::askForPath(bool displayWarning, const PathType &pathType)
{
    if (displayWarning) {
        QString mode = m_developmentType == DEBUG ? "DEBUG" : "RELEASE";

        if (pathType == PATH_LIB)
        QMessageBox::warning(this, tr("No ") + mode + tr(" mode library path"),
                             tr("This is likely the first time you use Papyrus in ") + mode + tr(" mode,"
                             " and you need to specify the path to alexandria's libs.\nA window will display"
                             ", allowing you to specify the directory."));
        else if (pathType == PATH_DESC)
            QMessageBox::warning(this, tr("No ") + mode + tr(" mode description path"),
                                         tr("This is likely the first time you use Papyrus in ") + mode + tr(" mode,"
                                         " and you need to specify the path to alexandria's description files.\n"
                                         "A window will display, allowing you to specify the directory."));
        else
            informUserAndCrash(tr("unsupported PathType when asking for path.\nSupported PathType "
                                  "are PATH_LIB and PATH_DESC"));
    }

    QString type;
    if (pathType == PATH_LIB)
        type = "library";
    else if (pathType == PATH_DESC)
        type = "description";
    else
        informUserAndCrash(tr("Unsupported PathType when asking for path.\nSupported PathType "
                              "are PATH_LIB and PATH_DESC"));

    if (m_developmentType == DEBUG) {
        QString ret = QFileDialog::getExistingDirectory(this,
                                                        tr("Provide ") + type + tr(" path for DEBUG mode"),
                                                        "/home",
                                                        QFileDialog::ShowDirsOnly);
        if (pathType == PATH_LIB)
            m_debugLibPath = ret;
        else if (pathType == PATH_DESC)
            m_debugPath = ret;
        else
            informUserAndCrash(tr("Unsupported PathType when asking for path.\nSupported PathType "
                                  "are PATH_LIB and PATH_DESC"));
    } else {
        QString ret = QFileDialog::getExistingDirectory(this,
                                                        tr("Provide ") + type + tr(" path for RELEASE mode"),
                                                        "/home",
                                                        QFileDialog::ShowDirsOnly);
        if (pathType == PATH_LIB)
            m_releaseLibPath = ret;
        else if (pathType == PATH_DESC)
            m_releasePath = ret;
        else
            informUserAndCrash(tr("Unsupported PathType when asking for path.\nSupported PathType "
                                  "are PATH_LIB and PATH_DESC"));
    }
}

/**
 * @brief PapyrusWindow::parseOneLevel parses all .xml description files at this level, and
 * recursively calls itself for all directories, looking again ofr .xml description files
 * @param dir
 */
void PapyrusWindow::parseOneLevel(QDir dir, XmlDescriptionReader *xmlReader)
{
    // First, parse descriptions files at the root of the category
    dir.setNameFilters(QStringList() << "*.xml"); // Match on XML files only

    QStringList descriptionFiles = dir.entryList();
    foreach (QString descFile, descriptionFiles) {
        QFile xmlFile(dir.absoluteFilePath(descFile));
        if (!xmlFile.open(QFile::ReadOnly | QFile::Text)) {
            qDebug() << "\t[X] Could not open file" << descFile << "for parsing";
            m_libraryParsingErrors += 1;
            break;
        }

        // Read the XML file
        if (!xmlReader->read(&xmlFile, xmlFile.fileName())) {
            m_libraryParsingErrors += 1;
            qWarning() << "\t[X] Failed to parse " << xmlFile.fileName();
        }
    }

    // Then, recurse in every directory
    dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot);
    QStringList subdirs = dir.entryList();
    foreach (QString subdir, subdirs) {
        parseOneLevel(QDir(dir.canonicalPath() + "/" + subdir), xmlReader);
    }
}

/**
 * @brief PapyrusWindow::getDescriptionPath returns either the debug or release description path,
 * bassed on the development type. this is simply a way to save keystrokes (we need the path at
 * several places, and makign the comparison each time is stupid)
 * @return
 */
QString PapyrusWindow::getDescriptionPath()
{
    if (m_developmentType == DEBUG)
        return m_debugPath;

    if (m_developmentType == RELEASE)
        return m_releasePath;

    informUserAndCrash(tr("Unsupported development type"),
                       tr("We could not parse the development type while trying to get the "
                          "description path. Currently supported is DEBUG or RELEASE. This is most"
                          " likely due to an API change that was not backported."));
}

QString PapyrusWindow::getLibPath()
{
    if (m_developmentType == DEBUG)
        return m_debugLibPath;

    if (m_developmentType == RELEASE)
        return m_releaseLibPath;

    informUserAndCrash(tr("Unsupported development type"),
                       tr("We could not parse the development type while trying to get the "
                          "library path. Currently supported is DEBUG or RELEASE. This is most"
                          " likely due to an API change that was not backported."));
}

/**
 * @brief PapyrusWindow::updateButtonsState is called when the user changes the active script. It
 * is used to display the buttons' (play, pause, connect, etc.) state accordingly to the new active
 * state. (One script might be launched and running, the other, not).
 */
void PapyrusWindow::updateButtonsState()
{
    // Make sure we have an active script
    if (m_activeScript == NULL)
        return;

    m_ui->actionRun->setEnabled(true);

    ROSSession *session = m_activeScript->rosSession();
    if (session == NULL) {
        displayStatusMessage(tr("Not ROS Session for script \"%1\"").arg(m_activeScript->name()), MSG_ERROR);
        return;
    }

    if (session->isRunning()) {
        m_ui->actionStop->setEnabled(true);
        m_ui->actionScope->setEnabled(true);

        if (!session->isPaused()) {
            m_ui->actionRun->setIcon(QIcon(":/icons/icons/pause.svg"));
            m_ui->actionRun->setToolTip(tr("Pause script"));
        } else {
            m_ui->actionRun->setIcon(QIcon(":/icons/icons/play.svg"));
            m_ui->actionRun->setToolTip(tr("Resume script"));
        }
    } else {
        m_ui->actionRun->setIcon(QIcon(":/icons/icons/play.svg"));
        m_ui->actionRun->setToolTip(tr("Launch script"));
        m_ui->actionStop->setEnabled(false);
        m_ui->actionScope->setEnabled(false);
    }
}

/**
 * @brief PapyrusWindow::categoryExpanded is called when a user double clicks on a category to
 * expand it, we use this event to collapse all other categories (expect "Constants") so that only
 * one category is active at a time (for improved readability)
 * @param item
 */
void PapyrusWindow::categoryExpanded(QTreeWidgetItem *item)
{
    // Don't do anything if the library search field is not empty (this is important because the
    // handler hides some categories, which would call this event)
    if (!librarySearchField_->text().isEmpty())
        return;

    Category *expandedCategory = dynamic_cast<Category *>(item);
    if (expandedCategory == NULL)
        return;

    // Don't do anything when expanding the "Constants" category
    if (expandedCategory->name() == "Constants")
        return;

    int n = libraryPanel_->topLevelItemCount();
    for (int i = 0; i < n; i += 1) {
        Category *cat = dynamic_cast<Category *>(libraryPanel_->topLevelItem(i));
        if (cat == NULL)
            continue;

        if (cat != expandedCategory && cat->name() != "Constants")
            cat->setExpanded(false);
    }

    // Save the last category that was expanded
    m_lastExpandedCategory = expandedCategory->name();
}

/**
 * @brief Fires when the current tab changes. Used to update the pointer to the current active
 *        script.
 * @param index: the newly active tab index
 */
void PapyrusWindow::on_tabWidget_currentChanged(int index)
{
    Q_UNUSED(index);

    // Check we have switched to the Home Page
    HomePage *homePage = dynamic_cast<HomePage *>(m_ui->tabWidget->currentWidget());
    // If we have, disable all buttons (and restore the "play" icon to the play/pause button)
    if (homePage != NULL) {
        m_ui->actionRun->setIcon(QIcon(":/icons/icons/play.svg"));
        m_ui->actionRun->setEnabled(false);
        m_ui->actionStop->setEnabled(false);
        m_ui->actionScope->setEnabled(false);
        if (m_runTimeDisplay != NULL) // this is null the first time, because its' not created yet
            m_runTimeDisplay->setEnabled(false);
        m_activeScript = NULL;
        return;
    }

    // Otherwise, try to get a DiagramView
    DiagramView *currentView = dynamic_cast<DiagramView *>(m_ui->tabWidget->currentWidget());
    // If there is none, there's an issue
    if (currentView == NULL) {
            m_ui->statusBar->showMessage(tr("Error when switching tab and trying to update active script "
                                            "(this is an internal error, you should report it.)"));
            m_activeScript = NULL;
            return;
    }

    // Get the scene associated with the view
    DiagramScene *currentScene = dynamic_cast<DiagramScene *>(currentView->scene());
    if (currentScene == NULL) {
        // TODO: _actually_ automatically report it instead of asking the user to do it.
        m_ui->statusBar->showMessage(tr("Error when switching tab and trying to update active script "
                                        "(this is an internal error, you should report it.)"));
        m_activeScript = NULL;
        return;
    }

    // De-active current script (is any)
    if (m_activeScript != NULL)
        m_activeScript->setIsActiveScript(false);

    // Get the script associated with the scene and set it as the active script
    m_activeScript = currentScene->script();
    m_activeScript->setIsActiveScript(true);

    // Update the buttons state to match the new script's status)
    updateButtonsState();
}

void PapyrusWindow::on_tabWidget_tabBarDoubleClicked(int index)
{
    DiagramView *view = dynamic_cast<DiagramView *>(m_ui->tabWidget->widget(index));
    if (view == NULL) {
        m_ui->statusBar->showMessage(tr("Could not rename script: failed to get the associated view."));
        return;
    }

    DiagramScene *scene = dynamic_cast<DiagramScene *>(view->scene());
    if (scene == NULL) {
        m_ui->statusBar->showMessage(tr("Could not rename script: failed to get the associated scene."));
        return;
    }

    QString currentName = scene->script()->name();
    Script *script = scene->script();
    QString currentFilePath = script->filePath();

    QMessageBox msgBox;
    msgBox.setWindowTitle(tr("Papyrus - Rename script"));
    msgBox.setText(tr("Here you can rename script \"") + currentName + "\"");
    msgBox.setIcon(QMessageBox::Question);
    QGridLayout *layout = dynamic_cast<QGridLayout *>(msgBox.layout());
    if (layout == NULL)
        return;
    QFormLayout *addLayout = new QFormLayout();
    QLineEdit *newName = new QLineEdit(currentName);
    QLabel label(tr("New script name:"));
    QCheckBox *cBox = new QCheckBox(tr("Also rename file?"));
    cBox->setDisabled(currentFilePath.isEmpty()); // Don't offer to rename if no path exists
    addLayout->addRow(&label, newName);
    addLayout->addRow(cBox);
    layout->addLayout(addLayout, 1, 0, 1, 3);
    msgBox.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);

    m_ui->statusBar->showMessage(tr("Renaming \"") + currentName + "\"...");

    int ret = msgBox.exec();
    if (ret == QMessageBox::Ok) {
        QString newScriptName(newName->text());
        QString str(tr("\"") + currentName + "\" renamed to \"" + newScriptName + "\"");
        script->setName(newScriptName);
        m_ui->tabWidget->tabBar()->setTabText(index, newScriptName);

        if (cBox->isChecked() && !currentFilePath.isEmpty()) {
            QFileInfo fi(currentFilePath);
            QString ext = fi.completeSuffix();
            QFile file(currentFilePath);
            QString dir(fi.absoluteDir().absolutePath());
            QString sanitizedScriptName(newScriptName.toLower().replace(" ", "_"));
            QString newScriptFilePath(dir + "/" + sanitizedScriptName + "." + ext);

            script->setFilePath(newScriptFilePath);
            script->setStatusModified(true);

            if (file.rename(newScriptFilePath)) {
                str += tr(", and XML filed renamed too.");
            } else {
                str += tr(", BUT the XML file could NOT be renamed (reason unknown).");
            }

            m_propertiesPanel->displayScriptProperties(scene->script());
        } else {
            str += ".";
        }

        m_ui->statusBar->showMessage(str);
    } else {
        m_ui->statusBar->showMessage(tr("Renaming cancelled. Nothing was done."));
    }
}

void PapyrusWindow::on_actionClose_Script_triggered()
{
    DiagramView *view = dynamic_cast<DiagramView *>(m_ui->tabWidget->currentWidget());
    if (view == NULL) {
        m_ui->statusBar->showMessage(tr("Could not close script: no script open!"));
        return;
    }

    DiagramScene *scene = dynamic_cast<DiagramScene *>(view->scene());
    if (scene == NULL) {
        qFatal("Could not close script: failed to get the associated scene.");
        return;
    }

    int currIdx = m_ui->tabWidget->currentIndex();
    QString scriptName = scene->script()->name();

    // Check if the script has unsaved modifications
    if (scene->script()->modified()) {
        switch (QMessageBox::question(this, tr("Save unsaved script?"),
                                      tr("This script have unsaved changes that will be lost if you close it now.\n"
                                         "Do you want to save them?"),
                                      QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel)) {
        case QMessageBox::Cancel:
            m_ui->statusBar->showMessage(tr("Cancel closing."));
            break;

        case QMessageBox::Save:
            // Save this script
            scene->script()->save(getDescriptionPath());

            // Make another pass to check that the script was indeed saved
            if (scene->script()->modified()) {
                QMessageBox::warning(this, tr("Script still unsaved"),
                                     tr("This script was still nto saved, exit aborted."));
                break;
            }

            // Remove the tab containing this widget
            m_ui->tabWidget->removeTab(currIdx);

            // Destroy the view
            delete view;
            m_ui->statusBar->showMessage(tr("Script ") + scriptName + tr(" closed."));
            break;

        case QMessageBox::Discard:
            // Remove the tab containing this widget
            m_ui->tabWidget->removeTab(currIdx);

            // Destroy the view
            delete view;
            m_ui->statusBar->showMessage(tr("Script ") + scriptName + tr(" closed (changes discarded)"));
            break;

        default:
            m_ui->statusBar->showMessage(tr("Cancel closing."));
        }
    } else {
        // Close the script directly if it has no unsaved changes
        // Remove the tab containing this widget
        m_ui->tabWidget->removeTab(currIdx);

        // Destroy the view
        delete view;
        m_ui->statusBar->showMessage(tr("Script ") + scriptName + tr(" closed."));
    }
}

/**
 * @brief PapyrusWindow::on_actionConnect_triggered shows a list of running kheops nodes and allow
 * the user to select one.
 * When a node is selected, it is asked for its script file, then it is opened and the ROS Session
 * can now be used to interact with the node.
 */
void PapyrusWindow::on_actionConnect_triggered()
{
    // Create an instance of NodesChooser, and make it modal to the application
    NodesChooser *nodesChooser = new NodesChooser;
    nodesChooser->setWindowFlag(Qt::Dialog);
    nodesChooser->setWindowModality(Qt::ApplicationModal);
    if (nodesChooser->exec()) {
        QString selectedNode = nodesChooser->selectedNode();

        if (!selectedNode.isEmpty()) {
            qDebug() << "Selected node:" << selectedNode;
            qDebug() << "Should create script now";
        }
    }

    /*
    // Make sure we do have an active script and its associated ROS Session
    if (m_activeScript == NULL) {
        displayStatusMessage(tr("No active script: cannot connect/disconnect"), MSG_ERROR);
        return;
    }

    if (m_activeScript->rosSession() == NULL) {
        displayStatusMessage(tr("No ROS session for the active script: cannot connect/disconnect"),
                             MSG_ERROR);
        return;
    }

    // CONNECT FUNCTION
    if (!m_activeScript->rosSession()->isConnected()) {
        // Create an instance of NodesChooser, and make it modal to the application
        NodesChooser *nodesChooser = new NodesChooser;
        nodesChooser->setWindowFlag(Qt::Dialog);
        nodesChooser->setWindowModality(Qt::ApplicationModal);

        // If we validated (and thus chose a node), transform the "connect" button into a "disconnect"
        if (nodesChooser->exec()) {
            // Update node name and connected status
            QString selectedNode = nodesChooser->selectedNode();
            m_activeScript->rosSession()->setNodeName(selectedNode);
            m_activeScript->rosSession()->setIsConnected(true);

            // Change the "connect" button into a "disconnect"
            m_ui->actionConnect->setIcon(QIcon(":/icons/icons/disconnect.svg"));
            m_ui->actionConnect->setToolTip(tr("Disconnect"));

            // If the node is the current script, then it's not running
            if (selectedNode == "Current Script") {
                m_rosSession->setIsRunning(false);
                m_rosSession->setIsPaused(false);

                // Make the play/pause button a "play" one
                m_ui->actionRun->setIcon(QIcon(":/icons/icons/play.svg"));
                m_ui->actionRun->setToolTip(tr("Resume script"));
                m_ui->actionRun->setEnabled(true);
                m_runTimeDisplay->setEnabled(true);

                // Disable the stop button
                m_ui->actionStop->setEnabled(false);
            }
            // Otherwise, we should ask the node its status (when kheops/#12 is solved)
            else {
                // Ask the script its status (paused or running)
                switch (m_rosSession->queryScriptStatus()) {
                case SCRIPT_RUNNING:
                    // Mark the script as running and not paused
                    m_rosSession->setIsRunning(true);
                    m_rosSession->setIsPaused(false);

                    // Make the play button into a "pause" one
                    m_ui->actionRun->setIcon(QIcon(":/icons/icons/pause.svg"));
                    m_ui->actionRun->setToolTip(tr("Pause script"));
                    m_ui->actionRun->setEnabled(true);
                    m_runTimeDisplay->setEnabled(true);

                    // Enable the stop button
                    m_ui->actionStop->setEnabled(true);
                    break;

                case SCRIPT_PAUSED:
                    //Mark the script as running and paused
                    m_rosSession->setIsRunning(true);
                    m_rosSession->setIsPaused(true);

                    // Make the play button into a "play" one
                    m_ui->actionRun->setIcon(QIcon(":/icons/icons/play.svg"));
                    m_ui->actionRun->setToolTip(tr("Resume script"));
                    m_ui->actionRun->setEnabled(true);
                    m_runTimeDisplay->setEnabled(true);

                    // Enable the stop button
                    m_ui->actionStop->setEnabled(true);
                    break;
                }
            }
        }
    } else {
        // DISCONNECT FUNCTION
        m_rosSession->setIsConnected(false);
        m_rosSession->setIsRunning(false);
        m_rosSession->setIsPaused(false);
        m_rosSession->setNodeName("");
        m_ui->actionConnect->setIcon(QIcon(":/icons/icons/connect.svg"));
        m_ui->actionConnect->setToolTip(tr("Connect"));
        m_ui->actionRun->setEnabled(false);
        m_ui->actionStop->setEnabled(false);
        m_ui->actionScope->setEnabled(false);
        m_runTimeDisplay->setEnabled(false);
    }
    //*/
}

void PapyrusWindow::on_actionRun_triggered()
{
    // Make sure we do have an active script and its associated ROS Session
    if (m_activeScript == NULL) {
        displayStatusMessage(tr("No active script: cannot play/pause"), MSG_ERROR);
        return;
    }

    if (m_activeScript->rosSession() == NULL) {
        displayStatusMessage(tr("No ROS session for the active script: cannot play/pause"),
                             MSG_ERROR);
        return;
    }

    // If the node is not already running (meaning it was not one we connected to), we need to have
    // the path of the library in order to launch a kheops instance
    if (!m_activeScript->rosSession()->isRunning()) {
        if (m_developmentType == RELEASE && m_releaseLibPath.isEmpty()) {
            askForPath(true, PATH_LIB);
            // Check that the user did specify a path and not cancelled
            if (m_releaseLibPath.isEmpty()) {
                displayStatusMessage(tr("Cannot launch script: you did not specify a lib path"),
                                     MSG_ERROR);
                return;
            }

        }
        else if (m_developmentType == DEBUG && m_debugLibPath.isEmpty()) {
            askForPath(true, PATH_LIB);
            // Check that the user did specify a path and not cancelled
            if (m_debugLibPath.isEmpty()) {
                displayStatusMessage(tr("Cannot launch script: you did not specify a lib path"),
                                     MSG_ERROR);
                return;
            }
        }

    }
    m_activeScript->rosSession()->runOrPause();
}

void PapyrusWindow::on_actionStop_triggered()
{
    // Make sure we do have an active script and its associated ROS Session
    if (m_activeScript == NULL) {
        displayStatusMessage(tr("No active script: cannot stop"), MSG_ERROR);
        return;
    }

    if (m_activeScript->rosSession() == NULL) {
        displayStatusMessage(tr("No ROS session for the active script: cannot stop"),
                             MSG_ERROR);
        return;
    }
    m_activeScript->rosSession()->stop();
}

void PapyrusWindow::on_actionScope_triggered()
{
    // Make sure we do have an active script and its associated ROS Session
    if (m_activeScript == NULL) {
        displayStatusMessage(tr("No active script: cannot scope"), MSG_ERROR);
        return;
    }

    if (m_activeScript->rosSession() == NULL) {
        displayStatusMessage(tr("No ROS session for the active script: cannot scope"),
                             MSG_ERROR);
        return;
    }

    displayStatusMessage(tr("Action scope not implemented yet"), MSG_WARNING);
}

void PapyrusWindow::on_actionEdit_paths_triggered()
{
    qDebug() << "Should edit path";
}
