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

#include "xmlscriptreader.h"
#include "helpers.h"
#include "constants.h"
#include "constantdiagrambox.h"
#include "activityvisualizer.h"
#include "activityvisualizerbars.h"
#include "activityvisualizerthermal.h"
#include "activityfetcher.h"

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
						m_script->scene()->addItem(link->label());
						link->updateLines();
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
			// NOTE: this is code boilerplate, because this is basicaly the body of the function
			// DiagramScene::checkForInvalidLinks() (with 'upateLines()' added). I decided not to
			// call the function because it would re-traverse all items a second time, while it's
			// virtually free to do here.
			// TODO: get rid of the need to call Link::updateLines() (this is a dirty hack tbh) and
			// at this point, we will replace those lines by a call to this function.
			bool foundInvalidReason = false;
			foreach (QGraphicsItem *i, m_script->scene()->items()) {
				Link *link= dynamic_cast<Link *>(i);
				DiagramBox *box = dynamic_cast<DiagramBox *>(i);
				if (link != nullptr) {
					link->updateLines();
					// NOTE: keep this order, because if you swap and 'foundInvalidLinks' is true,
					// then C++ will apply lazy evaluation and not evaluate the call to link->checkIfInvalid()
					// which will result in the links not being checked!
					foundInvalidReason = link->checkIfInvalid() || foundInvalidReason;
				} else if (box != nullptr) {
					foundInvalidReason = box->checkIfBoxInvalid() || foundInvalidReason;
				}
			}
			scene->update();
			m_script->setIsInvalid(foundInvalidReason);
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
	QString description;
	MatrixShape matrixShape = SHAPE_NONE;
	bool save = false;
	OutputSlot *outputSlot = new OutputSlot;
	int rows = 1;
	int cols = 1;
	std::vector<InputSlot *> inputSlots;
	QUuid uuid;
	QString topic;
	QString iconFilePath = ":icons/icons/missing-icon.svg";
	bool publish;
	bool createVisualizer = false;
	bool visuVisible;
	QPointF visuPos;
	QSizeF visuSize;
	VisuType visuType = NONE;
	QMap<QString, QVariant> parameters;
	bool isCommented = false; // defaults to non commented

	readUUID(&uuid);
	readCommented(isCommented);

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
		else if (reader.name() == "parameters")
			readParameters(parameters);
		else if (reader.name() == "visuType")
			readVisuType(visuType);
		else if (reader.name() == "visualizer")
			readVisualizer(createVisualizer, visuVisible, visuPos, visuSize);
		else if (reader.name() == "libname")
			reader.skipCurrentElement(); // Silently skip <libname>
		else {
			qWarning() << QString("Unsupported tag <%1>, skipping").arg(reader.name().toString());
			reader.skipCurrentElement();
		}
	}

	// We should check (somehow) that the parsing for this box was okay before adding it

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
				functionFound = true;

				// Extract DiagramBox's information
				iconFilePath = f->iconFilepath();
				description = f->description();
				libname = f->libName();
				matrixShape = f->matrixShape();

				// Extract InputSlots' information
				foreach (InputSlot *iSlot, inputSlots) {
					if (iSlot == nullptr) {
						qWarning() << "Null-pointer found in set of input slots";
						continue;
					}


					// Look for the same input slot in the library's Function
					foreach (InputSlot *fISlot, f->inputs()) {
						if (fISlot == nullptr) {
							qWarning() << "Null-pointer found in set of input slots (in library)";
							continue;
						}

						// Update script's input slot with the library's information
						if (fISlot->name() == iSlot->name()) {
							iSlot->setMultiple(fISlot->multiple());
							iSlot->setInputType(fISlot->inputType());
							iSlot->setCheckSize(fISlot->checkSize());
							iSlot->setDescription(fISlot->description());
							iSlot->setMatrixShape(fISlot->matrixShape());
							break;
						}
					}
				}
				break;
			}
		}
	}

	// We need to single out the inhib input slot
	std::vector<InputSlot *> inputSlotsWithoutInhib;
	InhibInput *inhib = nullptr;
	foreach (InputSlot *iSlot, inputSlots) {
		if (iSlot->name() != INHIBITION_INPUT_NAME)
			inputSlotsWithoutInhib.push_back(iSlot);
		else {
			inhib = dynamic_cast<InhibInput *>(iSlot);
			if (inhib == nullptr) {
				informUserAndCrash(QObject::tr("Failed to cast InputSlot to InhibInput!"));
			}
		}
	}

	DiagramBox *b;
	if (reader.name() == "constant")
		b = new ConstantDiagramBox(name, outputSlot, description, uuid);
	else {
		b = new DiagramBox(name, outputSlot, inputSlotsWithoutInhib, description, uuid, inhib);

		b->setTitle(title);
		b->setLibname(libname);
		b->setSaveActivity(save);
		b->setPublish(publish);
		if (!topic.isEmpty())
			b->setTopic(topic);
		b->setIsCommented(isCommented);
	}
	if(visuType == NONE)
		visuType = UNKNOWN;
	b->setIconFilepath(iconFilePath);
	b->setRows(rows);
	b->setCols(cols);
	b->setMatrixShape(matrixShape);
	b->setVisuType(visuType);
	b->fillVisuParameters(parameters);
	m_script->scene()->addBox(b, pos);

	// Check whether we should create the visualizer, and if yes, check which one
	// WARNING: this is code duplication from diagramscene.cpp, we should factor out this code!
	if (createVisualizer && reader.name() != "constant") {
		if(is2DVisuType(visuType))
		{
			ActivityVisualizer *vis = nullptr;
			switch (b->outputType()) {
				case SCALAR:
					vis = new ActivityVisualizerBars(b);
				break;

				case MATRIX:
					// (1,1) matrix is treated as a scalar
					if (b->rows() == 1 && b->cols() == 1)
						vis = new ActivityVisualizerBars(b);
					// (1,N) and (N,1) are vectors: they are displayed as several scalars
					else if (b->rows() == 1 || b->cols() == 1)
						vis = new ActivityVisualizerBars(b);
					else
						vis = new ActivityVisualizerThermal(b);
				break;

				default:
					qDebug() << "Ouput type not supported for visualization";
				return;
				break;
			}

			if (vis != nullptr) {
				vis->setPos(visuPos);
				vis->setWidth(visuSize.width());
				vis->setHeight(visuSize.height());
				vis->setVisible(visuVisible);
				vis->updatePixmap();

				// Create the activity fetcher with the topic name
				ActivityFetcher *fetcher = nullptr;
				if (b->publish()) {
					fetcher = new ActivityFetcher(b->topic(), b);
				} else {
					fetcher = new ActivityFetcher(ensureSlashPrefix(mkTopicName(b->scriptName(),
					                                                            b->uuid().toString())),
					                              b);
					m_script->rosSession()->addToHotList(QSet<QUuid>() << b->uuid());
				}

				// This is dirty, but this is used to trigger the correct onSizeChanged() event (the
				// child's one, not the mother class) to repaint correctly the axes, the function name,
				// etc. This must be refactored to be cleaner!
				ActivityVisualizerBars *visBars = dynamic_cast<ActivityVisualizerBars *>(vis);
				if (visBars != nullptr) {
					visBars->onSizeChanged();
					visBars->setActivityFetcher(fetcher);
					QObject::connect(fetcher, SIGNAL(newMatrix(QVector<qreal>*)), visBars, SLOT(updateBars(QVector<qreal>*)));
				} else {
					ActivityVisualizerThermal *visTh = dynamic_cast<ActivityVisualizerThermal *>(vis);
					if (visTh != nullptr) {
						visTh->onSizeChanged();
						visTh->setActivityFetcher(fetcher);
						QObject::connect(fetcher, SIGNAL(newMatrix(QVector<qreal>*)), visTh, SLOT(updateThermal(QVector<qreal>*)));
					}
					else
						qWarning() << "Only Bars and Thermal visualizers are supported for now!";
				}

			}
		}
		else if(is3DVisuType(visuType))
		{
			ShaderWidget *widget = createShaderWidget(visuType, b->getRows(), b->getCols(), parameters);
			ShaderMoveBar *shaderMoveBar = new ShaderMoveBar();
			ShaderProxy *proxy = new ShaderProxy(widget, shaderMoveBar, b);
			shaderMoveBar->setProxy(proxy);
			proxy->positionWidget(visuPos.x(), visuPos.y());
			proxy->resizeWidget(visuSize.width(), visuSize.height());
			QObject::connect(dynamic_cast<DiagramScene *>(b->scene()), SIGNAL(hideShaderWidgets()), proxy, SLOT(hideDisplay()));
			QObject::connect(dynamic_cast<DiagramScene *>(b->scene()), SIGNAL(showShaderWidgets()), proxy, SLOT(showDisplay()));

			b->scene()->addItem(shaderMoveBar);
			b->setDisplayedProxy(proxy);

			// Create the activity fetcher with the topic name
			ActivityFetcher *fetcher = nullptr;
			if (b->publish()) {
				fetcher = new ActivityFetcher(b->topic(), b);
			} else {
				fetcher = new ActivityFetcher(ensureSlashPrefix(mkTopicName(b->scriptName(),
				                                                            b->uuid().toString())),
				                              b);
				m_script->rosSession()->addToHotList(QSet<QUuid>() << b->uuid());
			}
			proxy->setActivityFetcher(fetcher);
			QObject::connect(fetcher, SIGNAL(newMatrix(QVector<qreal>*)), proxy, SLOT(updateValues(QVector<qreal>*)));
			proxy->setVisible(visuVisible);
		}
		else
			qWarning() << "Unknown visualization type";
	}

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

/**
 * @brief XmlScriptReader::readInputSlots parses the <inputs> tag in XML script file.
 * @param inputSlots
 * @param allBoxes
 * @param incompleteLinks
 */
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
			InputSlot *inputSlot = nullptr;
			while (reader.readNextStartElement()) {
				if (reader.name() == "name") {
					QString inputName = reader.readElementText();
					if (inputName == INHIBITION_INPUT_NAME)
						inputSlot = new InhibInput;
					else
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
						m_script->scene()->addItem(link->label());
						link->updateLines();
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
				Connectivity connectivity = stringToConnectivity(reader.attributes().value("type").toString());
				link->setConnectivity(connectivity);

				// Parse the regexes for links with connectivity ONE_TO_NEI
				if (connectivity == ONE_TO_NEI) {
					QStringList regexes;
					while(reader.readNextStartElement()) {
						if (reader.name() == "expression") {
							regexes << reader.readElementText();
						} else {
							reader.skipCurrentElement();
						}
					}
					link->setRegexes(regexes.join('\n'));
				}
				// Otherwise, consume the tag (it's crucial: otherwise we can't parse the rest!)
				else
					reader.skipCurrentElement();
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
	zone->updateGroup();
}

void XmlScriptReader::readVisualizer(bool &createVisualizer, bool &visuVisible, QPointF &visuPos, QSizeF &visuSize)
{
	Q_ASSERT(reader.isStartElement() && reader.name() == "visualizer");

	while (reader.readNextStartElement()) {
		if (reader.name() == "visible") {
			QString visible = reader.readElementText();
			if (visible.toLower() == "true")
				visuVisible = true;
			else if (visible.toLower() == "false")
				visuVisible = false;
			else {
				reader.raiseError(QObject::tr("Invalid value for <visible> flag, accepted are 'true' and 'false'"));
				reader.skipCurrentElement();
			}
		} else if (reader.name() == "position") {
			while (reader.readNextStartElement()) {
				if (reader.name() == "x") {
					qreal x = reader.readElementText().toDouble();
					visuPos.setX(x);
				} else if (reader.name() == "y") {
					qreal y = reader.readElementText().toDouble();
					visuPos.setY(y);
				} else
					reader.skipCurrentElement();
			}
		} else if (reader.name() == "size") {
			while (reader.readNextStartElement()) {
				if (reader.name() == "width") {
					qreal width = reader.readElementText().toDouble();
					visuSize.setWidth(width);
				} else if (reader.name() == "height") {
					qreal height = reader.readElementText().toDouble();
					visuSize.setHeight(height);
				} else
					reader.skipCurrentElement();
			}
		} else {
			reader.skipCurrentElement();
		}
	}

	// For now, if this function is called, it means we will create the visualizer.
	// But we need to make additional checks to be sure it's valid/
	// This is also true for every parsing function here :/
	createVisualizer = true;
}

/**
 * @brief XmlScriptReader::readCommented try to read the "commented" attribute if present, and
 * defaults to false otherwise
 * @param isCommented
 */
void XmlScriptReader::readCommented(bool &isCommented)
{
	Q_ASSERT(reader.isStartElement() && reader.name() == "function");

	// Check if there is attribute, and parse if found
	if (reader.attributes().hasAttribute("commented")) {
		QString v = reader.attributes().value("commented").toString();

		if (v == "false")
			isCommented = false;
		else if (v == "true")
			isCommented = true;
		else
			reader.raiseError(QObject::tr("Invalid attribute value for 'commented', accepted is 'true' or 'false'"));
	}
}

void XmlScriptReader::readVisuType(VisuType &visuType)
{
	visuType = stringToVisuType(reader.readElementText());
}

void XmlScriptReader::readParameters(QMap<QString, QVariant> &parameters)
{
	while (reader.readNextStartElement())
	{
		parameters.insert(reader.name().toString(), QVariant(reader.readElementText()));
	}
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

