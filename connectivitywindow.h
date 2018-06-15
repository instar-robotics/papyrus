#ifndef CONNECTIVITYWINDOW_H
#define CONNECTIVITYWINDOW_H

#include <QTabWidget>

namespace Ui {
class ConnectivityWindow;
}

class ConnectivityWindow : public QTabWidget
{
    Q_OBJECT

public:
    explicit ConnectivityWindow(QWidget *parent = 0);
    ~ConnectivityWindow();

private:
    Ui::ConnectivityWindow *ui;
};

#endif // CONNECTIVITYWINDOW_H
