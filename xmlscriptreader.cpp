#include "xmlscriptreader.h"
#include "helpers.h"
#include "constants.h"

#include <QDebug>
#include <iostream>

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

// TODO: it should be a while(readNextElement()) and use decidated functions such as
// readRTToken(), readScene(), readScriptName(), etc. (and the order should maybe not matter?)
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

            // Then read the rt_token flag
            if (reader.readNextStartElement()) {
                if (reader.name() == "rt_token") {
                    // Read its UUID
                    if (reader.attributes().hasAttribute("uuid")) {
                        QUuid uuid(reader.attributes().value("uuid").toString());
                        if (!uuid.isNull()) {
                            m_script->setUuid(uuid);
                        } else {
                            reader.raiseError(QObject::tr("Invalid UUID for RT Token."));
                        }
                    } else {
                        reader.raiseError(QObject::tr("Missing UUID attribute for the RT Token."));
                    }

                    // Read its unit
                    if (reader.attributes().hasAttribute("unit")) {
                        QString unit(reader.attributes().value("unit").toString());
                        if (!unit.isNull() && (unit == "Hz" || unit == "ms")) {
                            if (unit == "Hz")
                                m_script->setTimeUnit(HZ);
                            else
                                m_script->setTimeUnit(MS);
                        } else {
                            reader.raiseError(QObject::tr("Invalid unit for the RT Token."));
                        }
                    } else {
                        reader.raiseError(QObject::tr("Missing unit attribute for the RT Token."));
                    }

                    // Read its value
                    bool ok = false;
                    double timeValue = reader.readElementText().toDouble(&ok);
                    if (ok)
                        m_script->setTimeValue(timeValue);
                    else
                        reader.raiseError(QObject::tr("Invalid unit value for the RT Token."));
                } else {
                    reader.raiseError(QObject::tr("No RT Token found."));
                }
            }

            // Then read the scene's rect
            if (reader.readNextStartElement()) {
                if (reader.name() == "scene") {
                    double x, y, width, height;

                    while (reader.readNextStartElement()) {
                        if (reader.name() == "x")
                            x = reader.readElementText().toDouble();
                        else if (reader.name() == "y")
                            y = reader.readElementText().toDouble();
                        else if (reader.name() == "width")
                            width = reader.readElementText().toDouble();
                        else if (reader.name() == "height")
                            height = reader.readElementText().toDouble();
                        else
                            reader.skipCurrentElement();
                    }

                    m_script->scene()->setSceneRect(QRectF(x, y, width, height));
                } else {
                    reader.raiseError(QObject::tr("No scene found."));
                }
            }

            // Then read the opening <functions> tag
            if (reader.readNextStartElement()) {
                if (reader.name() == "functions") {
                    /*
                     * Then loop through all <function> tags. We create the function (box) when we
                     * parse it, and add a link to it in the dict below.
                     * When we find a link for its input(s), we lookup the uuid in the dict and see
                     * if the box it originates from was already created.
                     * If yes, make the link, otherwise, store the (incomplete) link in the second
                     * dict. When <function> parsing is over, we traverse the incomplete links are
                     * complete them.
                     */
                    std::map<QUuid, DiagramBox *> allBoxes;
                    std::map<QUuid, Link *> incompleteLinks;
                    while (reader.readNextStartElement()) {
                        if (reader.name() == "function" || reader.name() == "constant")
                            readFunction(&allBoxes, &incompleteLinks);
                        else
                            reader.skipCurrentElement();
                    }

                    if (incompleteLinks.size() > 0) {
                        foreach (auto pair, incompleteLinks) {
                            QUuid fromUuid = pair.first;
                            Link *link = pair.second;

                            std::map<QUuid, DiagramBox *>::iterator it = allBoxes.find(fromUuid);

                            if (it != allBoxes.end()) {
                                OutputSlot *oSlot = it->second->outputSlot();
                                link->setFrom(oSlot);
                                oSlot->addOutput(link);
                                m_script->scene()->addItem(link);
                                link->addLinesToScene();

                                if (link->checkIfInvalid())
                                    m_script->setIsInvalid(true);
                                link->setZValue(LINKS_Z_VALUE);
                                incompleteLinks.erase(fromUuid);
                            } else {
                                informUserAndCrash(QObject::tr("One incomplete link could not be "
                                                               "completed: did not find its "
                                                               "originating box."));
                            }
                        }
                    }
                    DiagramScene *scene = m_script->scene();
//                    scene->updateSceneRect();
                    scene->update();
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

void XmlScriptReader::readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
                                   std::map<QUuid, Link *> *incompleteLinks)
{
    Q_ASSERT(reader.isStartElement() && (reader.name() == "function" || reader.name() == "constant"));

    QPointF pos;
    QString name;
    bool save = false;
    QString descriptionFile;
    OutputSlot *outputSlot = new OutputSlot;
    int rows = 0;
    int cols = 0;
    std::set<InputSlot *> inputSlots;
    QUuid uuid;

    readUUID(&uuid);
    while (reader.readNextStartElement()) {
        if (reader.name() == "name")
            readFunctionName(&name);
        else if (reader.name() == "save")
            readFunctionSave(&save);
        else if (reader.name() == "inputs")
            readInputSlots(&inputSlots, allBoxes, incompleteLinks);
        else if (reader.name() == "output")
            readOutputSlot(outputSlot, &rows, &cols);
        else if (reader.name() == "position")
            readPosition(&pos);
        else if (reader.name() == "description")
            readDescription(&descriptionFile);
        else {
            reader.skipCurrentElement();
        }
    }

    // We should check (somehow) that the parsing for this box was okay before adding it
    // The Icon is not yet passed in the XML, so add a temporary default icon
    // Try to find
    QIcon icon(descriptionFile);
//    DiagramBox *b = m_script->scene()->addBox(pos, name, icon, outputSlot, inputSlots, descriptionFile, uuid);
    DiagramBox *b = new DiagramBox(name, icon, outputSlot, inputSlots, uuid);
    b->setDescriptionFile(descriptionFile);
    b->setSaveActivity(save);
    b->setRows(rows);
    b->setCols(cols);
    m_script->scene()->addBox(b, pos);

    if (reader.name() == "constant")
        b->setConstant(true);

    // Add this box to the dict
    allBoxes->insert(std::pair<QUuid, DiagramBox *>(uuid, b));

    QString iconPath(descriptionFile);
    iconPath.replace(".xml", ".svg");
    QFile f(iconPath);

    // Load icon from icon path if it exists, set missing icon otherwise
    QIcon neuralIcon;
    if (f.exists())
        neuralIcon = QIcon(iconPath);
    else
        neuralIcon = QIcon(":/icons/icons/missing-icon.svg");

    b->setIcon(neuralIcon);
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

void XmlScriptReader::readFunctionSave(bool *save)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "save");

    QString functionSave = reader.readElementText().toLower();

    if (functionSave.isEmpty()) {
        reader.raiseError(QObject::tr("Empty function save."));
    } else {
        *save = (functionSave == "true") ? true : false;
    }
}

void XmlScriptReader::readInputSlots(std::set<InputSlot *> *inputSlots,
                                     std::map<QUuid, DiagramBox *> *allBoxes,
                                     std::map<QUuid, Link *> *incompleteLinks)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "inputs");

    while (reader.readNextStartElement()) {
        if (reader.name() == "input") {
            QXmlStreamAttributes attributes = reader.attributes();

            if (!attributes.hasAttribute("type") || !attributes.hasAttribute("multiple") || !attributes.hasAttribute("uuid")) {
                reader.raiseError(QObject::tr("Missing attributes 'multiple', 'type' or 'uuid' to '<input>'."));
                reader.skipCurrentElement();
                continue;
            }

            InputType iType = stringToInputType(attributes.value("type").toString());

            bool multiple;
            QString multipleStr = attributes.value("multiple").toString().toLower();
            if (multipleStr == "true")
                multiple = true;
            else if (multipleStr == "false")
                multiple = false;
            else {
                reader.raiseError(QObject::tr("Failed to parse boolean value for 'multiple'"));
                reader.skipCurrentElement();
                continue;
            }

            QString strUuid = attributes.value("uuid").toString();

            // WARNING: this will segfault when <links> are before <name>
            InputSlot *inputSlot = NULL;
            while (reader.readNextStartElement()) {
                if (reader.name() == "name") {
                    QString inputName = reader.readElementText();
                    inputSlot = new InputSlot(inputName);
                    inputSlot->setMultiple(multiple);
                    inputSlot->setInputType(iType);
                    inputSlot->setUuid(QUuid(strUuid));
                    inputSlots->insert(inputSlot);
                } else if (reader.name() == "links") {
                    readLinks(inputSlot, allBoxes, incompleteLinks);
                } else
                    reader.skipCurrentElement();
            }
        } else
            reader.skipCurrentElement();
    }
}

void XmlScriptReader::readOutputSlot(OutputSlot *outputSlot, int *rows, int *cols)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "output" && reader.attributes().hasAttribute("type"));
    OutputType oType = stringToOutputType(reader.attributes().value("type").toString());

    outputSlot->setOutputType(oType);

    QString name;

    while (reader.readNextStartElement()) {
        name = reader.name().toString();

        if (name == "name") {
            QString outputName = reader.readElementText();
            outputSlot->setName(outputName);
        } else if (name == "rows") {
            *rows = reader.readElementText().toInt();
        } else if (name == "cols") {
            *cols = reader.readElementText().toInt();
        } else {
            reader.skipCurrentElement();
        }
    }
}

void XmlScriptReader::readUUID(QUuid *uuid)
{
    Q_ASSERT(reader.isStartElement() && (reader.name() == "function" || reader.name() == "constant")
             && reader.attributes().hasAttribute("uuid"));

    QUuid id(reader.attributes().value("uuid").toString());

    *uuid = id;
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

void XmlScriptReader::readLink(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
                               std::map<QUuid, Link *> *incompleteLinks)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "link");

    QXmlStreamAttributes attributes = reader.attributes();

    // Create the new link, without an originating output slot yet
    Link *link = new Link(NULL, inputSlot);

    if (attributes.hasAttribute("uuid")) {
        QUuid uuid(attributes.value("uuid").toString());
        if (!uuid.isNull()) {
            link->setUuid(uuid);
        } else {
            reader.raiseError(QObject::tr("Invalid UUID for a link."));
        }
    } else {
        reader.raiseError(QObject::tr("Missing UUID attribute for a link."));
    }

    if (attributes.hasAttribute("secondary")) {
        // TODO: check if parsed value is "true" or "false" and invalid on anything else
        QString isSecondary(attributes.value("secondary").toString());
        link->setSecondary(isSecondary == "true" ? true : false);
    } else {
        reader.raiseError(QObject::tr("Missing secondary attribute for a link."));
    }

    while (reader.readNextStartElement()) {
        if (reader.name() == "weight") {
            bool ok = false;
            double weight = reader.readElementText().toDouble(&ok);
            if (ok) {
                link->setWeight(weight);
            } else {
                reader.raiseError(QObject::tr("Invalid weight for a type."));
            }
        } else if (reader.name() == "from") {
            QUuid fromUuid(reader.readElementText());

            if (!fromUuid.isNull()) {
                std::map<QUuid, DiagramBox *>::iterator it = allBoxes->find(fromUuid);
                // Now we check if the box with this UUID was already created (and so we can create
                // the link now) or not (in which case we store the link in the imcomplete dict)
                if (it != allBoxes->end()) {
                    DiagramBox *fromBox = it->second;
                    OutputSlot *fromSlot = fromBox->outputSlot();
                    if (fromSlot != NULL) {
                        link->setFrom(fromSlot);
                        fromSlot->addOutput(link);
                        // Should add to scene here
                        m_script->scene()->addItem(link);
                        link->addLinesToScene();
                        if (link->checkIfInvalid())
                            m_script->setIsInvalid(true);
                        link->setZValue(LINKS_Z_VALUE);
                        link->updateLines();
                        m_script->scene()->update();
                    } else {
                        // If the OutputSlot is not yet created, we can't link, so put in incomplete
                        incompleteLinks->insert(std::pair<QUuid, Link *>(fromUuid, link));
                    }
                } else {
                    // If the box doesn't already exists, store it in incomplete
                    incompleteLinks->insert(std::pair<QUuid, Link *>(fromUuid, link));
                }
            } else {
                reader.raiseError(QObject::tr("Invalid UUID in from field for a link."));
            }
        } else {
            reader.skipCurrentElement();
        }
    }
}


void XmlScriptReader::readDescription(QString *descriptionFile)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "description");

    QString description = reader.readElementText();

    if (description.isEmpty()) {
        reader.raiseError(QObject::tr("Empty description file."));
    } else {
        descriptionFile->setUnicode(description.unicode(), description.size());
    }
}

void XmlScriptReader::readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
                                std::map<QUuid, Link *> *incompleteLinks)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "links");

    while (reader.readNextStartElement()) {
        if (reader.name() == "link")
            readLink(inputSlot, allBoxes, incompleteLinks);
        else
            reader.skipCurrentElement();
    }
}

