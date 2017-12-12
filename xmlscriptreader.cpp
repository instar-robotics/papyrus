#include "xmlscriptreader.h"

XmlScriptReader::XmlScriptReader(Script *script) : m_script(script)
{

}

bool XmlScriptReader::read(QIODevice *device)
{
    reader.setDevice(device);

    if (reader.readNextStartElement()) {
        if (reader.name() == "script") {
            readScript();
        } else {
            reader.raiseError(QObject::tr("Not a script file."));
        }
    } else {
        reader.raiseError(QObject::tr("Empty script file."));
    }

    if (reader.error())
        m_errorString = reader.errorString();

    return !reader.error();
}

QString XmlScriptReader::errorString() const
{
    return m_errorString;
}

void XmlScriptReader::readScript()
{
    // First read the name of the script
    if (reader.readNextStartElement()) {
        if (reader.name() == "name") {
            QString scriptName = reader.readElementText();

            if (scriptName.isEmpty()) {
                reader.raiseError(QObject::tr("Empty script name."));
            } else {
                m_script->setName(scriptName);
            }

            // Then read the opening <functions> tag
            if (reader.readNextStartElement()) {
                if (reader.name() == "functions") {
                    // Then loop through all the <function> tags
                    while (reader.readNextStartElement()) {
                        if (reader.name() == "function")
                            readFunction();
                        else
                            reader.skipCurrentElement();
                    }
                } else {
                    reader.raiseError(QObject::tr("Missing functions definition."));
                }
            } else {
                reader.raiseError(QObject::tr("Incomplete script file (failed to parse functions)."));
            }
        } else {
            reader.raiseError(QObject::tr("Missing script name."));
        }
    } else {
        reader.raiseError(QObject::tr("Incomplete script file (failed to parse name)."));
    }
}

void XmlScriptReader::readFunction()
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "function");

    QPointF pos;
    QString name;

    while (reader.readNextStartElement()) {
        if (reader.name() == "name")
            readFunctionName(&name);
        else if (reader.name() == "position")
            readPosition(&pos);
        else
            reader.skipCurrentElement();
    }

    // We should check (somehow) that the parsing for this box was okay before adding it
    // The Icon is not yet passed in the XMl, so add a temporary default icon
    m_script->scene()->addBox(pos, name, QIcon(":/icons/icons/missing-icon.svg"));
}

void XmlScriptReader::readFunctionName(QString *name)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "name");

    QString functionName = reader.readElementText();

    if (functionName.isEmpty()) {
        reader.raiseError(QObject::tr("Empty function name."));
    } else {
        name->setUnicode(functionName.unicode(), functionName.size());
    }
}

void XmlScriptReader::readPosition(QPointF *pos)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "position");

    while (reader.readNextStartElement()) {
        if (reader.name() == "x") {
            qreal x = reader.readElementText().toDouble();
            pos->setX(x);
        } else if (reader.name() == "y") {
            qreal y = reader.readElementText().toDouble();
            pos->setY(y);
        } else
            reader.skipCurrentElement();
    }
}

