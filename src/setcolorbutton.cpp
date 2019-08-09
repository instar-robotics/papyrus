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

#include "setcolorbutton.h"

#include <QColorDialog>
#include <QDebug>

SetColorButton::SetColorButton(QWidget *parent) : QPushButton(parent), m_color(qRgba(51, 153, 255, 10))
{
	connect(this, SIGNAL(clicked()), this, SLOT(changeColor()));
}

void SetColorButton::updateColor()
{
	setStyleSheet("background-color: " + m_color.name());
}

QColor SetColorButton::color() const
{
	return m_color;
}

void SetColorButton::setColor(const QColor &color)
{
	m_color = color;
	updateColor();
}

void SetColorButton::setCurrentScene(DiagramScene *currentScene)
{
	m_currentScene = currentScene;
}


void SetColorButton::changeColor()
{
	//If opengl widgets are visible in the current scene, hide them while the user chose the color
	if (m_currentScene != nullptr)
		m_currentScene->hide3DVisualizations();

	QColor newColor = QColorDialog::getColor(m_color,
	                                         parentWidget(),
	                                         tr("Change comment zone color"),
	                                         QColorDialog::ShowAlphaChannel);
	if (newColor != m_color)
		setColor(newColor);

	// Once the choice color window has been quit, set visible every widgets that have been hide just before
	if(m_currentScene != nullptr)
		m_currentScene->show3DVisualizations();
}
