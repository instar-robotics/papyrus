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

#ifndef XMLSCRIPTREADER_H
#define XMLSCRIPTREADER_H

#include "script.h"
#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"
#include "library.h"

#include <QXmlStreamReader>
#include <QUuid>

/**
 * @brief The XmlScriptReader class is an XML parser that reads an XML @Script file
 * to populate the @GraphicsScene.
 */

class XmlScriptReader
{
public:
	explicit XmlScriptReader(Script *script, const QString &descriptionPath, Library *library);
	bool read(QIODevice *device);

	QString errorString() const;

	QPointF centerView() const;
	void setCenterView(const QPointF &centerView);

	Library *library() const;
	void setLibrary(Library *library);

private:
	QXmlStreamReader reader;
	QString m_errorString;
	Script *m_script;
	QString m_descriptionPath;
	QPointF m_centerView;
	Library *m_library;

	void readScript();
	void readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
	                  std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readFunctionName(QString &name);
	void readFunctionTitle(QString &title);
	void readFunctionSave(bool *save);
	void readPublishTopic(QString &topic, bool *publish);
	void readInputSlots(std::vector<InputSlot *> *inputSlots,
	                    std::map<QUuid, DiagramBox *> *allBoxes,
	                    std::set<std::pair<QUuid, Link *>> *incompleteLinks);
	void readOutputSlot(OutputSlot *outputSlot, int *rows, int *cols,
	                    QString &rowsVariable, QString &colsVariable, bool &useValue);
	void readUUID(QUuid *uuid);
	void readPosition(QPointF *pos);
	void readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	               std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readLink(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	              std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readZone();
	void readVariables();
	void readVisualizer(bool &createVisualizer, bool &visuVisible, QPointF &visuPos, QSizeF &visuSize);
	void readCommented(bool &isCommented);
};

#endif // XMLSCRIPTREADER_H
