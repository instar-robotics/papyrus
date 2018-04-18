#include "xmlscriptreader.h"
#include "helpers.h"

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
                    /*
                     * Then loop through all the <function> tags.
                     * When we find a link, we store the IDs of the functions in a pair
                     * and once all functions are inserted in the scene, we add the links.
                     */
                    std::set<std::pair<QUuid, QUuid>> links;
                    while (reader.readNextStartElement()) {
                        if (reader.name() == "function")
                            readFunction(&links);
                        else
                            reader.skipCurrentElement();
                    }

                    /*
                     * Now create all links
                     * NOTE: right now the set of links is naively implemented: links are just
                     * pushed in, without any ordering. This means thant in order to create all
                     * links, we have to search the QGraphicsScene again and again, which is
                     * inefficient.
                     * A better way would be to store a map whose keys are the originating box and
                     * whose value is a set of all target functions.
                     * This way, the QGraphicsScene would be searched a fewer number of times (since
                     * we could keep a reference to the originating function).
                     * This is not a problem in the beginning, but it might be a bit slow when we
                     * reach a big number of boxes and a big number of links
                     */
                    DiagramScene *scene = m_script->scene();

                    // I don't like 'auto', but syntax error when using correct type
                    foreach (auto link, links) {
                        const QUuid originID = link.first;
                        const QUuid targetID = link.second;

                        DiagramBox *origin = NULL;
                        DiagramBox *target = NULL;

                        /*
                         * As explained before, this is suboptimal because we traverse the scene
                         * many times :/
                         */
                        foreach (QGraphicsItem *item, scene->items()) {
                            // Stop searching if we have found both boxes
                            if (origin != NULL && target != NULL)
                                break;

                            DiagramBox *box = dynamic_cast<DiagramBox *>(item);
                            if (box == NULL) {
                                // Not a box, so skip it
                                continue;
                            }

                            // Match the box against the origin and targets
                            if (box->uuid() == originID)
                                origin = box;

                            if (box->uuid() == targetID)
                                target = box;
                        }

                        /*
                         * If either 'origin' or 'target' is still NULL here, it means the link
                         * references an non-existing box. So issue a warning on console bar and
                         * skip it
                         */
                        if (origin == NULL || target == NULL) {
                            std::cerr << "Found a link that references a non-existing box ; "
                                         "skipping it." << std::endl;
                            continue;
                        }

                        // Actually create the arrow
                        QPointF startPoint = origin->scenePos();
                        startPoint.rx() += origin->boundingRect().right();
                        startPoint.ry() += origin->boundingRect().bottom() / 2;

                        QPointF endPoint = target->scenePos();
                        endPoint.ry() += target->boundingRect().bottom() / 2;

                        // TODO: create a function to "add an arrow" instead of doing this
                        // TODO : if still using this snippet, implement issue #2 to be able to set FROM and TO
                        Arrow *arrow = new Arrow(QLineF(startPoint, endPoint));
                        // Link the newly-created Arrow with its corresponding DiagramBoxes
//                        origin->addStartLine(arrow);
//                        target->addEndLine(arrow);
//                        arrow->setFrom(origin);
//                        arrow->setTo(target);

                        scene->addItem(arrow);
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

void XmlScriptReader::readFunction(std::set<std::pair<QUuid, QUuid> > *links)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "function");

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
            readInputSlots(&inputSlots);
        else if (reader.name() == "output")
            readOutputSlot(outputSlot, &rows, &cols);
        else if (reader.name() == "position")
            readPosition(&pos);
        else if (reader.name() == "link")
            readLink(uuid, links);
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
    DiagramBox *b = m_script->scene()->addBox(pos, name, icon, outputSlot, inputSlots, descriptionFile, uuid);
    b->setSaveActivity(save);
    b->setRows(rows);
    b->setCols(cols);

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

void XmlScriptReader::readInputSlots(std::set<InputSlot *> *inputSlots)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "inputs");

    while (reader.readNextStartElement()) {
        if (reader.name() == "input") {
            QXmlStreamAttributes attributes = reader.attributes();

            if (!attributes.hasAttribute("type") || !attributes.hasAttribute("multiple")) {
                reader.raiseError(QObject::tr("Missing attributes 'multiple' or 'type' to '<input>'."));
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

            while (reader.readNextStartElement()) {
                if (reader.name() == "name") {
                    QString inputName = reader.readElementText();
                    InputSlot *inputSlot = new InputSlot(inputName);
                    inputSlot->setMultiple(multiple);
                    inputSlot->setInputType(iType);
                    inputSlots->insert(inputSlot);
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
    Q_ASSERT(reader.isStartElement() && reader.name() == "function"
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

void XmlScriptReader::readLink(QUuid uuid, std::set<std::pair<QUuid, QUuid> > *links)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "link" && !uuid.isNull());

    QUuid targetUuid(reader.readElementText());

    Q_ASSERT(!targetUuid.isNull());

    links->insert(std::make_pair(uuid, targetUuid));
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

