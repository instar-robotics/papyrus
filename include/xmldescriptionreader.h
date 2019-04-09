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

#ifndef XMLDESCRIPTIONREADER_H
#define XMLDESCRIPTIONREADER_H

#include "category.h"
#include "slot.h"

#include <QXmlStreamReader>

/**
 * @brief The XmlDescriptionReader class is a XML file parser whose task is to populate a @Library
 * object with the @Categorie  and descriptions of neural @Function s, read from their XML description
 * file.
 */

class XmlDescriptionReader
{
public:
	XmlDescriptionReader(Category *category );

	bool read(QIODevice *device, const QString &descriptionFile);
private:
//    Library *m_library;
	Category *m_category;
	QXmlStreamReader reader;

	void readDescription(const QString &descriptionFile);
	void readAllFunctions(const QString &libName, const QString &descriptionFile);
	void readOneFunction(const QString &libName, const QString &descriptionFile);
	void readName(Function *function);
	void readInputs(Function *function);
	QString readIcon();
	void readParameterName(Slot *paramSlot);
	void readParameterType(OutputSlot *paramSlot);
	void readParameterType(InputSlot *paramSlot);
	void readParameterDesc(InputSlot *paramSlot);
	void readOutput(Function *function);
	void readFunctionDesc(Function *function);
};

#endif // XMLDESCRIPTIONREADER_H
