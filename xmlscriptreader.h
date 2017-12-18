#ifndef XMLSCRIPTREADER_H
#define XMLSCRIPTREADER_H

#include "script.h"
#include "diagrambox.h"
#include "outputslot.h"

#include <QXmlStreamReader>
#include <QUuid>

/**
 * @brief The XmlScriptReader class is an XML parser that reads an XML script file
 * to populate the GraphicsScene.
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
    void readFunction(std::set<std::pair<QUuid, QUuid>> *links);
    void readFunctionName(QString *name);
    void readOutputSlot(OutputSlot *outputSlot);
    void readUUID(QUuid *uuid);
    void readPosition(QPointF *pos);
    void readLink(QUuid uuid, std::set<std::pair<QUuid, QUuid>> *links);
    void readDescription(QString *descriptionPath);
};

#endif // XMLSCRIPTREADER_H
