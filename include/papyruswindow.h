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
#include "xmldescriptionreader.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDir>
#include <QSystemTrayIcon>
#include <QLineEdit>
#include <QLabel>
#include <QAction>
#include <QTimer>

namespace Ui {
class PapyrusWindow;
}

// Define the type of development environment (and where to go look for libraries)
enum DevelopmentType {
	RELEASE,
	DEBUG
};
Q_DECLARE_METATYPE(DevelopmentType) // This allows convertion from/to QVariant

// Define if we are asking the user for the DESCRIPTION or the LIBRARY path
enum PathType {
	PATH_DESC,
	PATH_LIB
};
Q_DECLARE_METATYPE(PathType);

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

	void readSettings(QString &lastOpenedScripts, int *lastActiveScript);
	void writeSettings();
	Script *parseXmlScriptFile(const QString &scriptPath);
	void askForPath(bool displayWarning, const PathType &pathType);
	void parseOneLevel(QDir dir, XmlDescriptionReader *xmlReader);
	QString getDescriptionPath();
	QString getLibPath();
	void updateButtonsState();

	QDir description() const {return description_;}
	void setDescription(QDir description) {description_ = description;}

	Category *addTreeRoot(QString name);

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

	DevelopmentType developmentType() const;

	QString debugPath() const;

	QString releasePath() const;

	QString debugLibPath() const;

	QString releaseLibPath() const;

	QString keyFile() const;
	void setKeyFile(const QString &keyFile);

	QString ivFile() const;
	void setIvFile(const QString &ivFile);

	QString lastDir() const;
	void setLastDir(const QString &lastDir);

	QTimer *autoSaveTimer() const;
	void setAutoSaveTimer(QTimer *autoSaveTimer);

private:
	Ui::PapyrusWindow *m_ui;
	RosNode *m_rosnode;
	int m_argc;
	char **m_argv;
	QLabel *m_rosMasterStatus;
	LibraryPanel *libraryPanel_;
	QLineEdit *librarySearchField_;
	QString m_lastExpandedCategory;  // Name of the last category that was expanded before filtering
	int m_libraryParsingErrors;
	QDir description_;
	QSystemTrayIcon *trayIcon;
	Library *m_library;
	std::set<Script *> m_scripts;
	Script *m_activeScript;
	PropertiesPanel *m_propertiesPanel;
	HomePage *m_homePage;
	QLineEdit *m_runTimeDisplay;
	DevelopmentType m_developmentType;
	QAction *m_actionRelease;
	QAction *m_actionDebug;
	QString m_debugPath;        // Path where to search for description files in DEBUG mode
	QString m_releasePath;      // Path where to search for description files in RELEASE mode
	QString m_debugLibPath;     // Path where to search for library files in DEBUG mode
	QString m_releaseLibPath;   // Path where to search for library files in RELEASE mode
	QString m_keyFile;          // Path of the key file to crypt / decrypt scrip files
	QString m_ivFile;           // Path of the IV
	QString m_lastDir;          // Last directory visited for saving or loading files
	QTimer *m_autoSaveTimer;    // Timer to trigger auto save for scripts
	QString m_changelogVersion; // Used to know if we should show the changelog on launch

signals:
	void toggleDisplayGrid(bool);
	void launched();

private slots:
	void filterLibraryNames(const QString &text);
	void displayStatusMessage(const QString &text, MessageUrgency urgency = MSG_INFO);
	void onROSMasterChange(bool isOnline);
	void onScriptResumed();
	void onScriptPaused();
	void onScriptStopped();
	void updateStopWatch(int h, int m, int s, int ms);
	void updateDevelopmentEnvironment(QAction *action);
	void categoryExpanded(QTreeWidgetItem *item);
	void autoSave();
	void onPropPanelEnter();
	void onPropPanelEscape();
	void onLaunched();
	void openScript(QString path = "");

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
	void on_actionShow_all_outputs_triggered();
	void on_actionHide_all_outputs_triggered();
	void on_actionList_shortcuts_triggered();
	void on_actionChangelog_triggered(bool isNewRelease = false);
	void on_actionReopen_last_scripts_triggered();
};

#endif // PAPYRUSWINDOW_H
