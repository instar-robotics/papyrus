/*
  Copyright (C) INSTAR Robotics

  Author: Nicolas SCHOEMAEKER

  This file is part of papyrus <https://github.com/instar-robotics/papyrus>.

  papyrus is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  papyrus is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/

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
