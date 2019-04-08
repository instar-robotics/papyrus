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

#ifndef FUNCTION_H
#define FUNCTION_H

#include "outputslot.h"
#include "inputslot.h"

#include <vector>

#include <QString>
#include <QIcon>
#include <QTreeWidgetItem>

/**
 * @brief The Function class describes a neural function for the @Library and @LibraryPanel
 * and comes from parsing a valid XML description file. Basically, the @Category class
 * holds several @Function.
 * The @Function class is only meant to be stored inside the @Library, but the actual neural
 * function is created as a @DiagramBox from this lightweight @Function object when it is
 * dropped on the @DiagramScene.
 */

class Function : public QTreeWidgetItem
{
public:
	Function(const QString &path);

	void updateTooltip();

	QString name() const;
	void setName(const QString &name);

	std::vector<InputSlot *> inputs() const;

	void addInputSlot (InputSlot *slot);

	OutputSlot *output() const;
	void setOutput(OutputSlot *output);

	QString descriptionFile() const;

	bool constant() const;
	void setConstant(bool constant);

	QString libName() const;
	void setLibName(const QString &libName);

	QString iconFilepath() const;
	void setIconFilepath(const QString &value);

	QString description() const;
	void setDescription(const QString &description);

	MatrixShape matrixShape() const;
	void setMatrixShape(const MatrixShape &matrixShape);

protected:
	QString m_name;
	QString m_descriptionFile;
	//    QIcon m_icon;
	std::vector<InputSlot *> m_inputs;
	OutputSlot *m_output;
	bool m_constant;   // Indicate whether this represents a constant input
	QString m_libName; // The name of the lib it belongs to, used for kheops to know where to look
	QString m_iconFilepath; // Filepath for the SVG icon, used when dropping the box on scene
	QString m_description;  // Description of the function
	MatrixShape m_matrixShape;
};

#endif // FUNCTION_H
