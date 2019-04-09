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


#include "nodeschooser.h"
#include "ui_nodeschooser.h"

#include "helpers.h"

NodesChooser::NodesChooser(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NodesChooser)
{
	ui->setupUi(this);

	populateKheopsNodes();

	// Make sure the first one is selected
	ui->comboBox->setCurrentIndex(0);

	m_selectedNode = ui->comboBox->currentText();
}

NodesChooser::~NodesChooser()
{
	delete ui;
}

void NodesChooser::populateKheopsNodes()
{
	// Populate the list of nodes upon creation
	QList<QString> kheopsNodes = getKheopsNodes();

	// Clear the list, and populate it with the nodes we just found
	ui->comboBox->clear();

	foreach (QString kheopsNode, kheopsNodes) {
		ui->comboBox->addItem(kheopsNode, kheopsNode);
	}
}

void NodesChooser::on_pushButton_clicked()
{
	ui->pushButton->setDisabled(true);
	populateKheopsNodes();
	ui->pushButton->setDisabled(false);
}

QString NodesChooser::selectedNode() const
{
	return m_selectedNode;
}

void NodesChooser::on_comboBox_currentTextChanged(const QString &arg1)
{
	m_selectedNode = arg1;
}
