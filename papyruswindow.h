#ifndef PAPYRUSWINDOW_H
#define PAPYRUSWINDOW_H

#include <QMainWindow>

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

private:
    Ui::PapyrusWindow *ui;
};

#endif // PAPYRUSWINDOW_H
