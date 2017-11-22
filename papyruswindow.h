#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>

namespace Ui {
class PapyrusWindow;
}

class PapyrusWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PapyrusWindow(QWidget *parent = 0);
    ~PapyrusWindow();

private slots:
    void on_actionExit_triggered();

    void on_btnNewScene_clicked();

    void on_actionAntialiasing_toggled(bool antialiasing);

    void on_actionZoom_In_triggered();

    void on_actionZoom_Out_triggered();

    void on_actionZoom_Fit_triggered();

private:
    Ui::PapyrusWindow *ui;
    int nbPage;
};

#endif // PAPYRUSWINDOW_H
