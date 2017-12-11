#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include "librarypanel.h"
#include "library.h"

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDir>
#include <QSystemTrayIcon>
#include <QLineEdit>

namespace Ui {
class PapyrusWindow;
}

class PapyrusWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PapyrusWindow(QWidget *parent = 0);
    ~PapyrusWindow();

    QDir description() const {return description_;}
    void setDescription(QDir description) {description_ = description;}

    Category *addTreeRoot(QString name);
    void addTreeChild(QTreeWidgetItem *parent, QIcon icon, QString name);

    QLineEdit *librarySearchField() const;
    void setLibrarySearchField(QLineEdit *librarySearchField);

    Ui::PapyrusWindow *getUi() const;

    Library *getLibrary() const;
    void setLibrary(Library *library);

signals:
    void toggleDisplayGrid(bool);

private slots:
    void filterLibraryNames(const QString &text);
    void displayStatusMessage(const QString &text);

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

private:
    Ui::PapyrusWindow *ui;
    LibraryPanel *libraryPanel_;
    QLineEdit *librarySearchField_;
    QDir description_;
    QSystemTrayIcon *trayIcon;
    Library *m_library;
};

#endif // PAPYRUSWINDOW_H
