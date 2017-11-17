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

    void zPlus();

    void zMinus();

    void selectionChanged();

private:
    Ui::PapyrusWindow *ui;
    int nbPage;
};

#endif // PAPYRUSWINDOW_H
