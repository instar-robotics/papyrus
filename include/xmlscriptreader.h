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
    explicit XmlScriptReader(Script *script);
    bool read(QIODevice *device);

    QString errorString() const;

private:
    QXmlStreamReader reader;
    QString m_errorString;
    Script *m_script;

    void readScript();
    void readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
                      std::map<QUuid, Link *> *incompleteLinks);
    void readFunctionName(QString &name);
    void readFunctionSave(bool *save);
    void readPublishTopic(QString &topic, bool *publish);
    void readInputSlots(std::set<InputSlot *> *inputSlots,
                        std::map<QUuid, DiagramBox *> *allBoxes,
                        std::map<QUuid, Link *> *incompleteLinks);
    void readOutputSlot(OutputSlot *outputSlot, int *rows, int *cols);
    void readUUID(QUuid *uuid);
    void readPosition(QPointF *pos);
    void readDescription(QString &descriptionPath);
    void readIcon(QString &iconFilepath);
    void readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
                   std::map<QUuid, Link *> *incompleteLinks);
    void readLink(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
                  std::map<QUuid, Link *> *incompleteLinks);
};

#endif // XMLSCRIPTREADER_H
