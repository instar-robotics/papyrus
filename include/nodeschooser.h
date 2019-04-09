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

#ifndef NODESCHOOSER_H
#define NODESCHOOSER_H

#include <QDialog>
#include <QDebug>
#include <QString>

#include <ros/ros.h>

namespace Ui {
class NodesChooser;
}

class NodesChooser : public QDialog
{
    Q_OBJECT

public:
    explicit NodesChooser(QWidget *parent = 0);
    ~NodesChooser();

    void populateKheopsNodes();

    QString selectedNode() const;

private slots:
    void on_pushButton_clicked();

    void on_comboBox_currentTextChanged(const QString &arg1);

private:
    Ui::NodesChooser *ui;
    QString m_selectedNode;
};

#endif // NODESCHOOSER_H
