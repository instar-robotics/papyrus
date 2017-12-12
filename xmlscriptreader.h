#ifndef XMLSCRIPTREADER_H
#define XMLSCRIPTREADER_H

#include "script.h"
#include "diagrambox.h"

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
    void readFunction();
    void readFunctionName(QString *name);
    void readUUID(QUuid *uuid);
    void readPosition(QPointF *pos);
};

#endif // XMLSCRIPTREADER_H
