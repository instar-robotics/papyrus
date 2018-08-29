#include "connectivitywindow.h"
#include "ui_connectivitywindow.h"

#include <QDebug>
#include <QGridLayout>
#include <QToolButton>

ConnectivityWindow::ConnectivityWindow(QSize inputSize, QSize outputSize, QWidget *parent) :
    QTabWidget(parent),
    ui(new Ui::ConnectivityWindow),
    m_inputSize(inputSize),
    m_outputSize(outputSize)
{
	ui->setupUi(this);

	QGridLayout *gridLayout = dynamic_cast<QGridLayout *>(ui->fromGroup->layout());
	if (gridLayout != NULL) {
		for(int i = 0; i < m_inputSize.width(); i += 1) {
			for (int j = 0; j < m_inputSize.height(); j += 1) {
//                QPushButton *btn = new QPushButton(QString::number(i+j));
				QToolButton *btn = new QToolButton;
				gridLayout->addWidget(btn, i, j);
			}
		}
	}

	gridLayout = dynamic_cast<QGridLayout *>(ui->toGroup->layout());
	if (gridLayout != NULL) {
		for(int i = 0; i < m_outputSize.width(); i += 1) {
			for (int j = 0; j < m_outputSize.height(); j += 1) {
//                QPushButton *btn = new QPushButton(QString::number(i+j));
				QToolButton *btn = new QToolButton;
				gridLayout->addWidget(btn, i, j);
			}
		}
	}

}

ConnectivityWindow::~ConnectivityWindow()
{
	delete ui;
}
