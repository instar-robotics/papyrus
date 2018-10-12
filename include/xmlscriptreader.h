#ifndef XMLSCRIPTREADER_H
#define XMLSCRIPTREADER_H

#include "script.h"
#include "diagrambox.h"
#include "outputslot.h"
#include "inputslot.h"

#include <QXmlStreamReader>
#include <QUuid>

/**
 * @brief The XmlScriptReader class is an XML parser that reads an XML @Script file
 * to populate the @GraphicsScene.
 */

class XmlScriptReader
{
public:
	explicit XmlScriptReader(Script *script, const QString &descriptionPath);
	bool read(QIODevice *device);

	QString errorString() const;

	QPointF centerView() const;
	void setCenterView(const QPointF &centerView);

private:
	QXmlStreamReader reader;
	QString m_errorString;
	Script *m_script;
	QString m_descriptionPath;
	QPointF m_centerView;

	void readScript();
	void readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
	                  std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readFunctionName(QString &name);
	void readFunctionLibname(QString &libname);
	void readFunctionSave(bool *save);
	void readPublishTopic(QString &topic, bool *publish);
	void readInputSlots(std::vector<InputSlot *> *inputSlots,
	                    std::map<QUuid, DiagramBox *> *allBoxes,
	                    std::set<std::pair<QUuid, Link *>> *incompleteLinks);
	void readOutputSlot(OutputSlot *outputSlot, int *rows, int *cols);
	void readUUID(QUuid *uuid);
	void readPosition(QPointF *pos);
	void readDescription(QString &descriptionPath);
	void readIcon(QString &iconFilepath);
	void readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	               std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readLink(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
	              std::set<std::pair<QUuid, Link *> > *incompleteLinks);
	void readZone();
};

#endif // XMLSCRIPTREADER_H
