#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDir>
#include <QSystemTrayIcon>
#include <QTreeWidget>

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

    QTreeWidgetItem *addTreeRoot(QString name);
    void addTreeChild(QTreeWidgetItem *parent, QIcon icon, QString name);

private slots:
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

private:
    Ui::PapyrusWindow *ui;
    QDir description_;
    QSystemTrayIcon *trayIcon;
};

#endif // PAPYRUSWINDOW_H
