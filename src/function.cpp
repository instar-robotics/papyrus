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

#include "function.h"
#include <QDebug>

Function::Function(const QString &path) : QTreeWidgetItem(),
                                          m_descriptionFile(path),
                                          m_constant(false),
                                          m_matrixShape(SHAPE_NONE)
{

}

Function::~Function()
{
	if (m_output != nullptr) {
		delete m_output;
		m_output = nullptr;
	}

	foreach (InputSlot *iSlot, m_inputs) {
		if (iSlot != nullptr)
			delete iSlot;
	}
}

/**
 * @brief Function::updateTooltip updates the tooltip with the function description and
 * all its slots's descrition
 */
void Function::updateTooltip()
{
	QString desc = "<h3>" + m_name + "</h3>";
	desc += "<p>" + m_description + "</p>";
	desc += "<p>Inputs:</p>";
	desc += "<ul>";
	foreach (InputSlot *input, m_inputs) {
		desc += "<li><strong>";
		desc += input->name() + ": </strong>";
		desc += input->description();
		desc += "</li>";
	}
	setToolTip(0, desc);
}

QString Function::name() const
{
	return m_name;
}

void Function::setName(const QString &name)
{
	m_name = name;
}

std::vector<InputSlot *> Function::inputs() const
{
	return m_inputs;
}

void Function::addInputSlot(InputSlot *slot)
{
	m_inputs.push_back(slot);
}

OutputSlot* Function::output() const
{
	return m_output;
}

void Function::setOutput(OutputSlot *output)
{
	m_output = output;
}

QString Function::descriptionFile() const
{
	return m_descriptionFile;
}

bool Function::constant() const
{
	return m_constant;
}

void Function::setConstant(bool constant)
{
	m_constant = constant;
}

QString Function::libName() const
{
	return m_libName;
}

void Function::setLibName(const QString &libName)
{
	m_libName = libName;
}

QString Function::iconFilepath() const
{
	return m_iconFilepath;
}

void Function::setIconFilepath(const QString &value)
{
	m_iconFilepath = value;
}

QString Function::description() const
{
	return m_description;
}

void Function::setDescription(const QString &description)
{
	m_description = description;
}

MatrixShape Function::matrixShape() const
{
	return m_matrixShape;
}

void Function::setMatrixShape(const MatrixShape &matrixShape)
{
	m_matrixShape = matrixShape;
}
