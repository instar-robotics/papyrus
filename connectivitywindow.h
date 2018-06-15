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
    explicit ConnectivityWindow(QSize inputSize, QSize outputSize, QWidget *parent = 0);
    ~ConnectivityWindow();

private:
    Ui::ConnectivityWindow *ui;
    QSize m_inputSize;   // Size of the input matrix
    QSize m_outputSize;  // Size of the output matrix
};

#endif // CONNECTIVITYWINDOW_H
