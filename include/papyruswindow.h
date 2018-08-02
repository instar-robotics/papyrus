#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include "librarypanel.h"
#include "library.h"
#include "script.h"
#include "propertiespanel.h"
#include "rosnode.h"
#include "homepage.h"
#include "rossession.h"
#include "types.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDir>
#include <QSystemTrayIcon>
#include <QLineEdit>
#include <QLabel>
#include <QAction>

namespace Ui {
class PapyrusWindow;
}

// Define the type of development environment (and where to go look for libraries)
enum DevelopmentType {
    RELEASE,
    DEBUG
};
Q_DECLARE_METATYPE(DevelopmentType) // This allows convertion from/to QVariant

/**
 * @brief The PapyrusWindow class is the main window of the application.
 * It contains the list of open @Script s, the @Library of @Function s,
 * the @DiagramView s, etc.
 */
class PapyrusWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PapyrusWindow(int argc, char **argv, QWidget *parent = 0);
    ~PapyrusWindow();

    void closeEvent(QCloseEvent *evt);

    void readSettings();
    void writeSettings();
    Script *parseXmlScriptFile(const QString &scriptPath);
    void askLibraryPath(bool displayWarning = false);

    QDir description() const {return description_;}
    void setDescription(QDir description) {description_ = description;}

    Category *addTreeRoot(QString name);
    void addTreeChild(QTreeWidgetItem *parent, QIcon icon, QString name);

    QLineEdit *librarySearchField() const;
    void setLibrarySearchField(QLineEdit *librarySearchField);

    Ui::PapyrusWindow *ui() const;

    Library *getLibrary() const;
    void setLibrary(Library *library);

    std::set<Script *> getScripts() const;
    void addScript(Script *script);

    Script *activeScript() const;

    PropertiesPanel *propertiesPanel() const;
    void setPropertiesPanel(PropertiesPanel *propertiesPanel);

    QSystemTrayIcon *getTrayIcon() const;

    RosNode *rosnode() const;
    void setRosnode(RosNode *rosnode);

    void spawnRosNode();

    HomePage *homePage() const;
    void setHomePage(HomePage *homePage);

    ROSSession *rosSession() const;
    void setRosSession(ROSSession *rosSession);

    DevelopmentType developmentType() const;

    QString debugPath() const;

    QString releasePath() const;

private:
    Ui::PapyrusWindow *m_ui;
    RosNode *m_rosnode;
    int m_argc;
    char **m_argv;
    QLabel *m_rosMasterStatus;
    LibraryPanel *libraryPanel_;
    QLineEdit *librarySearchField_;
    QDir description_;
    QSystemTrayIcon *trayIcon;
    Library *m_library;
    std::set<Script *> m_scripts;
    Script *m_activeScript;
    PropertiesPanel *m_propertiesPanel;
    HomePage *m_homePage;
    QLineEdit *m_runTimeDisplay;
    ROSSession *m_rosSession;
    DevelopmentType m_developmentType;
    QAction *m_actionRelease;
    QAction *m_actionDebug;
    QString m_debugPath;              // Path where to search for description files in DEBUG mode
    QString m_releasePath;            // Path where to search for description files in RELEASE mode

signals:
    void toggleDisplayGrid(bool);

private slots:
    void filterLibraryNames(const QString &text);
    void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
    void onROSMasterChange(bool isOnline);
    void onScriptResumed();
    void onScriptPaused();
    void onScriptStopped();
    void updateStopWatch(int h, int m, int s, int ms);
    void updateDevelopmentEnvironment(QAction *action);

    void on_actionExit_triggered();

    void on_actionAntialiasing_toggled(bool antialiasing);

    void on_actionZoom_In_triggered();

    void on_actionZoom_Out_triggered();

    void on_actionZoom_Fit_triggered();

    void on_actionNew_script_hovered();

    void on_actionOpen_Script_hovered();

    void on_actionSave_Script_hovered();

    void on_actionZoom_In_hovered();

    void on_actionZoom_Out_hovered();

    void on_actionZoom_Fit_hovered();

    void on_actionNew_script_triggered();

    void on_actionDisplay_Grid_hovered();

    void on_actionDisplay_Grid_toggled(bool arg1);

    void on_actionAbout_Papyrus_triggered();

    void on_actionSave_Script_triggered();

    void on_actionOpen_Script_triggered();
    void on_tabWidget_currentChanged(int index);
    void on_tabWidget_tabBarDoubleClicked(int index);
    void on_actionClose_Script_triggered();
    void on_actionConnect_triggered();
    void on_actionRun_triggered();
    void on_actionStop_triggered();
    void on_actionScope_triggered();
    void on_actionEdit_paths_triggered();
};

#endif // PAPYRUSWINDOW_H
