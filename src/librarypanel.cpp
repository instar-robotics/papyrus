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

#include "librarypanel.h"
#include "constants.h"
#include "function.h"
#include "outputslot.h"
#include "inputslot.h"
#include "slot.h"
#include "constantfunction.h"

#include <iostream>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QDrag>
#include <QIcon>
#include <QDebug>

LibraryPanel::LibraryPanel(QWidget *parent) : QTreeWidget(parent)
{
	setDragEnabled(true);
	setColumnCount(1);        // Just the function's name (an icon is added)
	setHeaderHidden(true);    // Hide the header, we don't need it
	setAnimated(true);
	setIconSize(QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));
	setIndentation(0);
	setRootIsDecorated(true); // Try to show the little arrow (doesn't work)
}

void LibraryPanel::dragEnterEvent(QDragEnterEvent *evt)
{
	evt->accept();
}

void LibraryPanel::dragMoveEvent(QDragMoveEvent *evt)
{
	evt->accept();
}

void LibraryPanel::dropEvent(QDropEvent *evt)
{
	evt->accept();
}

void LibraryPanel::startDrag(Qt::DropActions)
{
	Function *item = static_cast<Function *>(currentItem());
	ConstantFunction *constantItem = dynamic_cast<ConstantFunction *>(item);
	QString iconFilepath = item->iconFilepath();
	QIcon icon = item->icon(0);
	QString name = item->name();
	QString descriptionFile = item->descriptionFile();
	QString libname = item->libName();
	MatrixShape matrixShape = item->matrixShape();
	QString description = item->description();

	OutputSlot *outputSlot = item->output();
	OutputType outputType = outputSlot->outputType();
	bool constant = (constantItem != NULL);

	std::vector<InputSlot *>inputSlots = item->inputs();
	int nbInputs = inputSlots.size();

	QByteArray itemData;
	QDataStream dataStream(&itemData, QIODevice::WriteOnly);

	dataStream << name << iconFilepath << icon << descriptionFile << (qint32)outputType
	           << constant << nbInputs << libname << (qint32)matrixShape << description;

	// Add all input slots' name, type, multiple and checkSize
	foreach(InputSlot *i, inputSlots) {
		dataStream << i->name() << (qint32)i->inputType() << i->multiple() << i->checkSize()
		           << (qint32)i->matrixShape();
	}

	QMimeData *mimeData = new QMimeData;
	mimeData->setData(LibraryPanel::libraryItemMimeType(), itemData);

	QDrag *drag = new QDrag(this);
	drag->setMimeData(mimeData);
	drag->setPixmap(icon.pixmap(QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE)));
	drag->exec(Qt::CopyAction);
}

