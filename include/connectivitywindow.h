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
