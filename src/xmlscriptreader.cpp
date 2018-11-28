#include "xmlscriptreader.h"
#include "helpers.h"
#include "constants.h"
#include "constantdiagrambox.h"

#include <QDebug>
#include <iostream>

XmlScriptReader::XmlScriptReader(Script *script, const QString &descriptionPath, Library *library) :
    m_script(script),
    m_descriptionPath(descriptionPath),
    m_centerView(0, 0),
    m_library(library)
{
	// Set script's default value to detect XML parsing errors
	m_script->setUuid(QUuid());
	m_script->setTimeValue(-1.0);
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

QPointF XmlScriptReader::centerView() const
{
	return m_centerView;
}

void XmlScriptReader::setCenterView(const QPointF &centerView)
{
	m_centerView = centerView;
}

Library *XmlScriptReader::library() const
{
	return m_library;
}

void XmlScriptReader::setLibrary(Library *library)
{
	m_library = library;
}

// TODO: it should be a while(readNextElement()) and use decidated functions such as
// readRTToken(), readScene(), readScriptName(), etc. (and the order should maybe not matter?)
void XmlScriptReader::readScript()
{
	// First read the name of the script
	while (reader.readNextStartElement()) {
		if (reader.name() == "name") {
			QString scriptName = reader.readElementText();
			if (scriptName.isEmpty()) {
				reader.raiseError(QObject::tr("Empty script name."));
			} else {
				m_script->setName(scriptName);
			}
		}
		// Then read the rt_token flag
		else if (reader.name() == "rt_token") {
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
		}

		// Then read the scene's rect
		else if (reader.name() == "scene") {
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
		}

		// Then read the view's center position
		else if (reader.name() == "view") {

			while (reader.readNextStartElement()) {
				if (reader.name() == "centerX")
					m_centerView.setX(reader.readElementText().toDouble());
				else if (reader.name() == "centerY")
					m_centerView.setY(reader.readElementText().toDouble());
				else
					reader.skipCurrentElement();
			}
		}


		// Then read the opening <functions> tag
		else if (reader.name() == "functions") {
			/*
			 * Then loop through all <function> tags. We create the function (box) when we
			 * parse it, and add a link to it in the dict below.
			 * When we find a link for its input(s), we lookup the uuid in the dict and see
			 * if the box it originates from was already created.
			 * If yes, make the link, otherwise, store the (incomplete) link in the second
			 * dict. When <function> parsing is over, we traverse the incomplete links and
			 * complete them.
			 */
			std::map<QUuid, DiagramBox *> allBoxes;
			std::set<std::pair<QUuid, Link *>> incompleteLinks;
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

						link->setZValue(LINKS_Z_VALUE);
						// TODO: empty the set as we go?
						//                                incompleteLinks.erase(fromUuid);
					} else {
						informUserAndCrash(QObject::tr(
						                       "Link with UUID %1 could not be completed, because it did not find "
						                       "its originating box (with UUID %2)").arg(link->uuid().toString())
						                   .arg(fromUuid.toString()));
					}
				}
			}
			DiagramScene *scene = m_script->scene();

			// Make sure all links are displayed correctly
			foreach (QGraphicsItem *i, m_script->scene()->items()) {
				Link *link= dynamic_cast<Link *>(i);
				if (link != nullptr) {
					link->updateLines();
				}
			}
			scene->update();
		}

		// Then read the comment zones
		else if (reader.name() == "zones") {
			while (reader.readNextStartElement()) {
				if (reader.name() == "zone" )
					readZone();
				else
					reader.skipCurrentElement();
			}
		}
	}

	// Check if we could read everything that is mandatory
	if (m_script->name().isEmpty())
		reader.raiseError(QObject::tr("Missing script name"));
	if (m_script->uuid().isNull())
		reader.raiseError(QObject::tr("Missing RT Token (or RT Token's UUID)"));
	if (m_script->timeValue() < 0)
		reader.raiseError(QObject::tr("Missing (or negative) RT token value"));
}

void XmlScriptReader::readFunction(std::map<QUuid, DiagramBox *> *allBoxes,
                                   std::set<std::pair<QUuid, Link *>> *incompleteLinks)
{
	Q_ASSERT(reader.isStartElement() && (reader.name() == "function" || reader.name() == "constant"));

	QPointF pos;
	QString name;
	QString title;
	QString libname;
	bool save = false;
	OutputSlot *outputSlot = new OutputSlot;
	int rows = 0;
	int cols = 0;
	std::vector<InputSlot *> inputSlots;
	QUuid uuid;
	QString topic;
	QString iconFilePath = ":icons/icons/missing-icon.svg";
	bool publish;

	readUUID(&uuid);

	// Note: we deliberately do not read the <libname>. As this is only useful for Kheops, we
	// instead parse the function box and fetch its libname from the @Library. This allows for a
	// function box to change its library without needed to change the scripts
	while (reader.readNextStartElement()) {
		if (reader.name() == "name")
			readFunctionName(name);
		else if (reader.name() == "title")
			readFunctionTitle(title);
		else if (reader.name() == "save")
			readFunctionSave(&save);
		else if (reader.name() == "publish")
			readPublishTopic(topic, &publish);
		else if (reader.name() == "inputs")
			readInputSlots(&inputSlots, allBoxes, incompleteLinks);
		else if (reader.name() == "output")
			readOutputSlot(outputSlot, &rows, &cols);
		else if (reader.name() == "position")
			readPosition(&pos);
		else if (reader.name() == "libname")
			reader.skipCurrentElement(); // Silently skip <libname>
		else {
			qWarning() << QString("Unsupported tag <%1>, skipping").arg(reader.name().toString());
			reader.skipCurrentElement();
		}
	}

	// We should check (somehow) that the parsing for this box was okay before adding it
	QIcon icon(":/icons/icons/missing-icon.svg");
	// TODO read Icon

	// Traverse the library to find the Function with the given name, and extract information from it
	if (m_library == nullptr)
		informUserAndCrash(QObject::tr("Could not parse script because no library provided!"));

	bool functionFound = false;
	foreach (Category *category, m_library->categories()) {
		if (functionFound)
			break;

		int nbFunctions = category->childCount();
		for (int i = 0; i < nbFunctions; i += 1) {
			Function *f = dynamic_cast<Function *>(category->child(i));
			if (f == nullptr) {
				qWarning() << "Found non Function item while traversing Category" << category->name();
				continue;
			}

			// If we found the function in the library, extract its information
			if (f->name() == name) {
				icon = QIcon(f->iconFilepath());
				iconFilePath = f->iconFilepath();
				libname = f->libName();
				functionFound = true;
				break;
			}
		}
	}

	DiagramBox *b;
	if (reader.name() == "constant")
		b = new ConstantDiagramBox(name, icon, outputSlot, uuid);
	else {
		b = new DiagramBox(name, icon, outputSlot, inputSlots, uuid);

		b->setTitle(title);
		b->setLibname(libname);
		b->setSaveActivity(save);
		b->setPublish(publish);
		if (!topic.isEmpty())
			b->setTopic(topic);
	}

	b->setIconFilepath(iconFilePath);
	b->setRows(rows);
	b->setCols(cols);
	m_script->scene()->addBox(b, pos);

	// TODO: check all links for invalidity and set script's invalidity

	// Add this box to the dict
	allBoxes->insert(std::pair<QUuid, DiagramBox *>(uuid, b));
}

void XmlScriptReader::readFunctionName(QString &name)
{
	Q_ASSERT(reader.isStartElement() && reader.name() == "name");

	QString functionName = reader.readElementText();

	if (functionName.isEmpty()) {
		reader.raiseError(QObject::tr("Empty function name."));
	} else {
		name = functionName;
	}
}

void XmlScriptReader::readFunctionTitle(QString &title)
{
	Q_ASSERT(reader.isStartElement() && reader.name() == "title");

	// We allow empty function title
	title = reader.readElementText();
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

void XmlScriptReader::readPublishTopic(QString &topic, bool *publish)
{
	Q_ASSERT(reader.isStartElement() && reader.name() == "publish");

	// Warning: read the attributes *before* reading the element text, because
	// it advances the parsing cursor and then you've lost the ability to read attributes
	if (reader.attributes().hasAttribute("topic")) {
		QString topicName = reader.attributes().value("topic").toString();
		if (!topicName.isEmpty())
			topic = topicName;
	}

	QString shouldPublish = reader.readElementText().toLower();
	if (shouldPublish == "true")
		*publish = true;
	else if (shouldPublish == "false")
		*publish = false;
	else {
		reader.raiseError(QObject::tr("Unsupported value for <publish> field"));
		reader.skipCurrentElement();
	}
}

void XmlScriptReader::readInputSlots(std::vector<InputSlot *> *inputSlots,
                                     std::map<QUuid, DiagramBox *> *allBoxes,
                                     std::set<std::pair<QUuid, Link *>> *incompleteLinks)
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
					inputSlots->push_back(inputSlot);
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
                               std::set<std::pair<QUuid, Link *>> *incompleteLinks)
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
		} else if (reader.name() == "value") {
			link->setValue(reader.readElementText());
		} else if (reader.name() == "from") {
			QUuid fromUuid(reader.readElementText());

			if (!fromUuid.isNull()) {
				std::map<QUuid, DiagramBox *>::iterator it = allBoxes->find(fromUuid);
				// Now we check if the box with this UUID was already created (and so we can create
				// the link now) or not (in which case we store the link in the incomplete dict)
				if (it != allBoxes->end()) {
					DiagramBox *fromBox = it->second;
					if (fromBox == NULL)
						informUserAndCrash(QObject::tr("Got a null pointer stored in the 'allBoxes' map"));

					OutputSlot *fromSlot = fromBox->outputSlot();

					if (fromSlot != NULL) {
						link->setFrom(fromSlot);
						fromSlot->addOutput(link);

						// Add the link to the scene
						m_script->scene()->addItem(link);
						link->addLinesToScene();
						link->setZValue(LINKS_Z_VALUE);
//						link->updateLines();
//						m_script->scene()->update();
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
		} else if (reader.name() == "connectivity") {
			if (reader.attributes().hasAttribute("type")) {
				link->setConnectivity(stringToConnectivity(reader.attributes().value("type").toString()));
				reader.skipCurrentElement(); // To consume the tag!
			} else {
				qWarning() << "Missing attribute 'type' for <connectivity>";
				reader.raiseError(QObject::tr("Missing attribute 'type' for <connectivity>"));
			}
		} else {
			reader.skipCurrentElement();
		}
	}
}

void XmlScriptReader::readZone()
{
	Q_ASSERT(reader.isStartElement() && reader.name() == "zone");

	QString title;
	qreal x = 0;
	qreal y = 0;
	qreal width = 100;
	qreal height = 100;
	int r = 100;
	int g = 100;
	int b = 100;
	int a = 100;


	while (reader.readNextStartElement()) {
		if (reader.name() == "title") {
			title = reader.readElementText();
		} else if (reader.name() == "x") {
			x = reader.readElementText().toDouble();
		} else if (reader.name() == "y") {
			y = reader.readElementText().toDouble();
		} else if (reader.name() == "width") {
			width = reader.readElementText().toDouble();
		} else if (reader.name() == "height") {
			height = reader.readElementText().toDouble();
		} else if (reader.name() == "color") {
			if (!reader.attributes().hasAttribute("red") ||
			    !reader.attributes().hasAttribute("green") ||
			    !reader.attributes().hasAttribute("blue") ||
			    !reader.attributes().hasAttribute("alpha")) {
				reader.raiseError(QObject::tr("Missing r,g,b or alpha component for the color"));
			} else {
				r = reader.attributes().value("red").toInt();
				g = reader.attributes().value("green").toInt();
				b = reader.attributes().value("blue").toInt();
				a = reader.attributes().value("alpha").toInt();
			}

			// IMPORTANT: consumes the <color> tag
			reader.readElementText();
		} else
			reader.skipCurrentElement();
	}

	Zone *zone = new Zone(x, y, width, height);
	zone->setColor(QColor(r, g, b, a));
	zone->setTitle(title);
	m_script->scene()->addItem(zone);
	zone->moveBy(0.1,0); // Dirty trick to trigger the itemChange() and snap position on the grid
}

void XmlScriptReader::readLinks(InputSlot *inputSlot, std::map<QUuid, DiagramBox *> *allBoxes,
                                std::set<std::pair<QUuid, Link *>> *incompleteLinks)
{
	Q_ASSERT(reader.isStartElement() && reader.name() == "links");

	while (reader.readNextStartElement()) {
		if (reader.name() == "link")
			readLink(inputSlot, allBoxes, incompleteLinks);
		else
			reader.skipCurrentElement();
	}
}

