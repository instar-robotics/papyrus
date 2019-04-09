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

#ifndef ROSNODE_H
#define ROSNODE_H

#include <QThread>

#include <ros/ros.h>

/**
 * @brief The RosNode class is used to handle ROS
 * operations
 */
class RosNode : public QThread
{
	Q_OBJECT
public:
	RosNode(int argc, char **argv);
	virtual ~RosNode();

	void init();
	void run() override;

	bool shouldQuit() const;
	void setShouldQuit(bool value);

private:
	int m_argc;
	char **m_argv;
	ros::Subscriber m_sub;
	bool m_shouldQuit;       // Used by Papyrus to cleanly exit the thread

signals:
	void rosMasterChanged(bool isOnline);
};

#endif // ROSNODE_H
