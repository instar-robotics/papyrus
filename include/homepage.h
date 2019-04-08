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

#ifndef HOMEPAGE_H
#define HOMEPAGE_H

#include <QFrame>
#include <QLabel>
#include <QComboBox>

class HomePage : public QFrame
{
    Q_OBJECT
public:
    explicit HomePage(QWidget *parent = nullptr);

    QLabel *rosMasterStatus() const;
    void setRosMasterStatus(QLabel *rosMasterStatus);

private:
    QLabel *m_title;
    QLabel *m_rosMasterStatus;
    QLabel *m_kNodesLabel;
    QComboBox *m_kheopsNodes;

signals:

public slots:
    void onRosMasterChange(bool isOnline);
};

#endif // HOMEPAGE_H
