#include "connectivitywindow.h"
#include "ui_connectivitywindow.h"

ConnectivityWindow::ConnectivityWindow(QWidget *parent) :
    QTabWidget(parent),
    ui(new Ui::ConnectivityWindow)
{
    ui->setupUi(this);
}

ConnectivityWindow::~ConnectivityWindow()
{
    delete ui;
}
