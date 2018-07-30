#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "constants.h"
#include "diagramscene.h"
#include "diagramview.h"
#include "diagrambox.h"
#include "xmldescriptionreader.h"
#include "xmlscriptreader.h"
#include "helpers.h"
#include "nodeschooser.h"

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

PapyrusWindow::PapyrusWindow(int argc, char **argv, QRect availableGeometry, QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::PapyrusWindow),
    m_argc(argc),
    m_argv(argv),
    m_rosMasterStatus(NULL),
    m_activeScript(NULL),
    m_propertiesPanel(NULL),
    m_homePage(NULL),
    m_runTimeDisplay(NULL),
    m_rosSession(NULL)
{
    bool libraryParsingError = false;

    m_ui->setupUi(this);

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

    m_ui->splitter->insertWidget(0, leftSplitter);

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
    m_runTimeDisplay = new QLineEdit("00:00:00");
    m_runTimeDisplay->setReadOnly(true);
    m_runTimeDisplay->setEnabled(false);
    m_runTimeDisplay->setMaximumWidth(128);
    m_runTimeDisplay->setAlignment(Qt::AlignCenter);
    m_runTimeDisplay->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    m_ui->mainToolBar->insertWidget(m_ui->actionStop, m_runTimeDisplay);

    // Crease the empty ROS Session
    m_rosSession = new ROSSession;
}

PapyrusWindow::~PapyrusWindow()
{
    delete m_ui;
    delete trayIcon;
    delete m_library;
    delete m_rosnode;
    delete m_rosMasterStatus;
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
                m_ui->statusBar->showMessage(tr("Cancel exit."));
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
                m_ui->statusBar->showMessage(tr("Discarding unsaved script."));
                qApp->exit();
                break;
            default:
                m_ui->statusBar->showMessage(tr("Cancel exit."));
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
    connect(newScript, SIGNAL(displayStatusMessage(QString)), this, SLOT(displayStatusMessage(QString)));
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
    std::cout << "Should filter on '" << qPrintable(text) << "'" << std::endl;
}

void PapyrusWindow::displayStatusMessage(const QString &text)
{
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

ROSSession *PapyrusWindow::rosSession() const
{
    return m_rosSession;
}

void PapyrusWindow::setRosSession(ROSSession *rosSession)
{
    m_rosSession = rosSession;
}

void PapyrusWindow::on_actionSave_Script_triggered()
{
    // Call the 'Save' function of the current script
    if (m_activeScript == NULL) {
        m_ui->statusBar->showMessage(tr("No open script to save."));
        QMessageBox::warning(this, tr("No open script to save"), tr("There is no scripts opened to save!"));
        return;
    }

    m_activeScript->save();
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
        // Check that we can read the key and IV
        std::string keyFile("/home/nschoe/workspace/qt/papyrus/key");
        std::string ivFile("/home/nschoe/workspace/qt/papyrus/iv");

        if (!fileExists(keyFile) || !fileExists(ivFile)) {
            QMessageBox::warning(NULL, tr("Encryption key and IV not found"),
                                 tr("We could not open this encrypted script file because either the"
                                    " key of the IV file could not be found."));
            return;
        }

        // Read and store key
        std::string key;
        CryptoPP::FileSource fkey(keyFile.c_str(), true,
                                  new CryptoPP::HexDecoder(
                                      new CryptoPP::StringSink(key)));

        // Read and store iv
        std::string iv;
        CryptoPP::FileSource fiv(ivFile.c_str(), true,
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

    Q_ASSERT(openScene->script() != NULL);

    openScript->setFilePath(scriptPath);

    // Parse the script XML file
    XmlScriptReader xmlReader(openScript);
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
 * @brief Fires when the current tab changes. Used to update the pointer to the current active
 *        script.
 * @param index: the newly active tab index
 */
void PapyrusWindow::on_tabWidget_currentChanged(int index)
{
    Q_UNUSED(index);

    // Get the view that is displayed in the tab
    DiagramView *currentView = dynamic_cast<DiagramView *>(m_ui->tabWidget->currentWidget());
    if (currentView == NULL) {
        // TODO: _actually_ automatically report it instead of asking the user to do it.
        m_ui->statusBar->showMessage(tr("Error when switching tab and trying to update active script "
                                      "(this is an internal error, you should report it."));
        m_activeScript = NULL;
        return;
    }

    // Get the scene associated with the view
    DiagramScene *currentScene = dynamic_cast<DiagramScene *>(currentView->scene());
    if (currentScene == NULL) {
        // TODO: _actually_ automatically report it instead of asking the user to do it.
        m_ui->statusBar->showMessage(tr("Error when switching tab and trying to update active script "
                                      "(this is an internal error, you should report it."));
        m_activeScript = NULL;
        return;
    }

    // Get the script associated with the scene and set it as the active script
    m_activeScript = currentScene->script();
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
            scene->script()->save();

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

void PapyrusWindow::on_actionConnect_triggered()
{
    // CONNECT FUNCTION
    if (!m_rosSession->isConnected()) {
        // Create an instance of NodesChooser, and make it modal to the application
        NodesChooser *nodesChooser = new NodesChooser;
        nodesChooser->setWindowFlag(Qt::Dialog);
        nodesChooser->setWindowModality(Qt::ApplicationModal);

        // If we validated (and thus chose a node), transform the "connect" button into a "disconnect"
        if (nodesChooser->exec()) {
            QString selectedNode = nodesChooser->selectedNode();
            m_rosSession->setNodeName(selectedNode);
            m_rosSession->setIsConnected(true);
            m_ui->actionConnect->setIcon(QIcon(":/icons/icons/disconnect.svg"));
            m_ui->actionConnect->setToolTip(tr("Disconnect"));
            m_ui->actionRun->setEnabled(true);
            m_ui->actionStop->setEnabled(true);
            m_ui->actionScope->setEnabled(true);
            m_runTimeDisplay->setEnabled(true);
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
}

void PapyrusWindow::on_actionRun_triggered()
{
    qDebug() << "RUN";
}

void PapyrusWindow::on_actionStop_triggered()
{
    qDebug() << "STOP";
}

void PapyrusWindow::on_actionScope_triggered()
{
    qDebug() << "SCOPE";
}
