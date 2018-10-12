#include "script.h"
#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "outputslot.h"
#include "helpers.h"
#include "constants.h"
#include "constantdiagrambox.h"
#include "zone.h"

#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QXmlStreamWriter>
#include <QDebug>
#include <QGraphicsView>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <cryptopp/filters.h>
#include <cryptopp/aes.h>
#include <cryptopp/modes.h>
#include <cryptopp/osrng.h>
#include <cryptopp/hex.h>
#include <cryptopp/files.h>


Script::Script(DiagramScene *scene, const QString &name) : m_scene(scene),
                                                           m_name(name),
                                                           m_modified(false),
                                                           m_isInvalid(false),
                                                           m_timeValue(10.0),
                                                           m_timeUnit(HZ),
                                                           m_encrypt(false),
                                                           m_isActiveScript(false)
{
	if (scene != NULL) {
		scene->setScript(this);
	}

	m_uuid = QUuid::createUuid();

	m_modifiedNotifTimer = new QTimer(this);
	connect(m_modifiedNotifTimer, SIGNAL(timeout()), this, SLOT(warnAboutModifiedScript()));

	// Create an associated ROS Session for this script and connect signals
	m_rosSession = new ROSSession(NULL, this);
	m_rosSession->setNodeName(QString("/kheops_%1").arg(m_name));
	connect(m_rosSession, SIGNAL(displayStatusMessage(QString,MessageUrgency)), this,
	        SLOT(onROSSessionMessage(QString,MessageUrgency)));
	connect(m_rosSession, SIGNAL(scriptResumed()), this, SLOT(onScriptResumed()));
	connect(m_rosSession, SIGNAL(scriptPaused()), this, SLOT(onScriptPaused()));
	connect(m_rosSession, SIGNAL(scriptStopped()), this, SLOT(onScriptStopped()));
}

/**
 * @brief Save the script in its file, this means serializing the contents of the scene to the
 * XML file.
 * @param descriptionPath: the path to the (release or debug) description path, so that we can
 * strip it off of paths to save everything in relative.
 */
void Script::save(const QString &descriptionPath, const QString &basePath)
{
	// Prevent saving when the script is in invalid state
	if (m_isInvalid || m_name == NEW_SCRIPT_DEFAULT_NAME) {
		QMessageBox::warning(NULL, tr("Saving not allowed in invalid state!"),
		                     tr("You cannot save the script at this time because it is currently "
		                        "in an invalid state.\nThis can mean that:\n"
		                        "  - some function boxes are linked with SCALAR_MATRIX but are "
		                        "not the same size\n"
		                        "  - some function boxes have negative sizes\n"
		                        "  - it is still named \"") + QString(NEW_SCRIPT_DEFAULT_NAME) + tr("\"\n"
		                        "  - etc."));
		return;
	}

	emit displayStatusMessage(tr("Saving \"") + m_name + "\"...", MSG_INFO);

	// Read keys if the script is specified to be encrypted
	if (m_encrypt) {
		PapyrusWindow *mainWin = getMainWindow();

		// Check that the user has filled in a key and iv path
		if (mainWin->keyFile().isEmpty() || mainWin->ivFile().isEmpty()) {
			QMessageBox::warning(NULL, tr("Missing crypto information"),
			                     tr("You are trying to encrypt and save a script file, for "
			                        "this, we need an encryption key and IV.\nWe "
			                        "detected that at least one is missing. You will be prompted for"
			                        " paths in the next window, read the dialog's titles in order to"
			                        " provide either the key or the IV (don't get them mixed!)."));
		}

		if (mainWin->keyFile().isEmpty()) {
			mainWin->setKeyFile(QFileDialog::getOpenFileName(mainWin, tr("Please provide the KEY file"),
			                                                 basePath,
			                                                 "Key files (*.*)"));
		}

		if (mainWin->ivFile().isEmpty()) {
			mainWin->setIvFile(QFileDialog::getOpenFileName(mainWin, tr("Please provide the IV file"),
			                                                basePath,
			                                                "IV files (*.*)"));
		}

		// Make another check to be sure the user did not cancel
		if (mainWin->keyFile().isEmpty() || mainWin->ivFile().isEmpty()) {
			QMessageBox::warning(NULL, tr("Saving aborted"),
			                     tr("The script was not saved because you failed to provide either "
			                        "the key or the IV file for encryption."));
			return;
		}

		std::string keyFile = mainWin->keyFile().toStdString();
		std::string ivFile = mainWin->ivFile().toStdString();

		// Prevent saving if the script should be encrypted but no key/iv could be found
		if (!fileExists(keyFile) || !fileExists(ivFile)) {
			QMessageBox::warning(NULL, tr("Encryption key and IV not found"), tr("Saving is not possible because you specified that this "
			                              "script should be encrypted on save, but we could not find"
			                              " either the key or the iV file (or both)!"));
			return;
		}

		// Read and store key
		CryptoPP::FileSource fkey(keyFile.c_str(), true,
		                        new CryptoPP::HexDecoder(
		                            new CryptoPP::StringSink(m_key)));

		// Read and store iv
		CryptoPP::FileSource fiv(ivFile.c_str(), true,
		                        new CryptoPP::HexDecoder(
		                            new CryptoPP::StringSink(m_iv)));

		Q_UNUSED(fkey);
		Q_UNUSED(fiv);
	}

	// First check if we have a filepath in which to save the script
	if (m_filePath.isEmpty()) {
		emit displayStatusMessage(QObject::tr("No file for ") + m_name + tr(", please select one..."), MSG_WARNING);

		QString savePath = QFileDialog::getSaveFileName(NULL,
		                             QObject::tr("Save as..."),
		                             basePath,
		                             QObject::tr("XML files (*.xml);; Crypted XML files (*.xml.crypted)"));

		// Abort if it's empty
		if (savePath.isEmpty()) {
			QString text(tr("Cancelled saving "));
			text += name();
			emit displayStatusMessage(text, MSG_INFO);
			return;
		}

		// Qt will ask confirmation if the file exists by default, so don't ask again

		// Check if the user manually entered extension ".xml" and add it otherwise.
		QFileInfo fi(savePath);
		if (fi.suffix().isEmpty())
			savePath.append(".xml");
		else if (fi.suffix().toLower() != "xml") {
			// This case means the user entered an extension that is not XML so fail
			QMessageBox::warning(NULL, QObject::tr("Only .xml file are supported"),
			                     QObject::tr("Scripts must be saved in .xml files, please select or "
			                                 "enter a valid .xml filename."));
			return;
		}

		setFilePath(savePath);
	}

	// At this point, we should have a filePath
	Q_ASSERT (!m_filePath.isEmpty());

	// Open the XML file
	QFile file(m_filePath);
	if (!file.open(QIODevice::WriteOnly)) {
		QMessageBox::warning(NULL, QObject::tr("Could not open XML file"),
		                     QObject::tr("An error occured whlie trying to open the XML file to "
		                                 "save the script. "));
		return;
	}

	// Create the XML stream on the file
	QXmlStreamWriter stream(&file);
	stream.writeStartDocument();             // start the XML document
	stream.setAutoFormatting(true);          // Make it human-readable
	stream.writeStartElement("script");      // Write the root tag
	stream.writeTextElement("name", name()); // Write the name of the script

	// Write the RT token parameters
	stream.writeStartElement("rt_token");
	stream.writeAttribute("uuid", m_uuid.toString());
	stream.writeAttribute("unit", timeUnitToString(m_timeUnit));
	stream.writeCharacters(QString::number(m_timeValue));
	stream.writeEndElement(); // rt_token

	// Write the scene and view coordinates
	stream.writeStartElement("scene");
	QRectF rect(scene()->sceneRect());
	stream.writeTextElement("x", QString::number(rect.x()));
	stream.writeTextElement("y", QString::number(rect.y()));
	stream.writeTextElement("width", QString::number(rect.width()));
	stream.writeTextElement("height", QString::number(rect.height()));
	stream.writeEndElement(); // scene

	// Write the scene's center point
	QList<QGraphicsView *> views = scene()->views();
	if (views.size() == 0)
		informUserAndCrash(tr("No views attached to the scene: cannot save."));
	else if (views.size() > 1)
		informUserAndCrash(tr("More than one view is attached to the scene: unsupported!"));
	QGraphicsView *mainView = views[0];
	QPointF viewCenter = mainView->mapToScene(mainView->viewport()->rect().center());
	stream.writeStartElement("view");
	stream.writeTextElement("centerX", QString::number(viewCenter.x()));
	stream.writeTextElement("centerY", QString::number(viewCenter.y()));
	stream.writeEndElement(); // view

	// Write all functions
	stream.writeStartElement("functions");

	// Traverse all items in the scene and store them
	std::vector<Zone *> zones;
	foreach (QGraphicsItem *i, m_scene->items()) {
		// For some reasons, it fails with 'qgraphicsitem_cast' even though 'type()' is reimplemented
		DiagramBox *item = dynamic_cast<DiagramBox *>(i);
		if (item == nullptr) {
			// If the item is not a function, check if this is a comment zone
			Zone *zone = dynamic_cast<Zone *>(i);
			if (zone != nullptr) {
				zones.push_back(zone);
			}

			// Skip the element (whether it's a zone of not)
			continue;
		}

		ConstantDiagramBox *constantItem = dynamic_cast<ConstantDiagramBox *>(item);

		QString name = item->name();
		QPointF pos = item->scenePos();
		QUuid uuid = item->uuid();
		QString descriptionFile = item->descriptionFile();
		QString iconFilepath = item->iconFilepath();
		std::vector<InputSlot *>inputSlots = item->inputSlots();
		bool constant = (constantItem != NULL);

		// Strip the description path prefix from paths, to make it relative, unless this is a
		// resource (and begins with ":")
		if (descriptionFile.startsWith(descriptionPath + "/"))
			descriptionFile.remove(descriptionPath + "/");
		else if (!descriptionFile.startsWith(":"))
			qWarning() << "Description file" << descriptionFile << "cannot be made relative "
			"(for function" << name << "). Saving as absolute, but this will NOT be portable.";


		if (iconFilepath.startsWith(descriptionPath + "/"))
			iconFilepath.remove(descriptionPath + "/");
		else if (!iconFilepath.startsWith(":"))
			qWarning() << "Icon file" << iconFilepath << "cannot be made relative "
			"(for function" << name << "). Saving as absolute, but this will NOT be portable.";

		Q_ASSERT(!name.isEmpty());

		// We need to distinguish functions and constants for Kehops
		if (constant)
			stream.writeStartElement("constant");
		else
			stream.writeStartElement("function");

		stream.writeAttribute("uuid", uuid.toString());
		stream.writeTextElement("name", name);

		// Skip irrelevant information for constant objects
		if (!constant) {
			stream.writeTextElement("libname", item->libname());
			stream.writeTextElement("save", item->saveActivity() ? "true" : "false");

			stream.writeStartElement("publish");
			stream.writeAttribute("topic", item->topic());
			stream.writeCharacters(item->publish() ? "true" : "false");
			stream.writeEndElement(); // publish

			// Save input slots
			stream.writeStartElement("inputs");
			foreach (InputSlot *inputSlot, inputSlots) {
				stream.writeStartElement("input");
				stream.writeAttribute("type", inputTypeToString(inputSlot->inputType()));
				stream.writeAttribute("multiple", inputSlot->multiple() ? "true" : "false");
				stream.writeAttribute("uuid", inputSlot->uuid().toString());

				stream.writeTextElement("name", inputSlot->name());

				stream.writeStartElement("links");

				foreach (Link* link, inputSlot->inputs()) {
					// Should loop for each connection
					stream.writeStartElement("link");
					bool isSecondary = link->to()->box() == link->from()->box();

					// Write attribute "constant" when the originating box is constant
					// We don't explicitly write "false" because this is rare and don't want to clutter
					if (dynamic_cast<ConstantDiagramBox *>(link->from()->box()) != NULL)
						//                if (link->from()->box()->constant())
						                stream.writeAttribute("constant", "true");
					stream.writeAttribute("uuid", link->uuid().toString());
					stream.writeAttribute("secondary", isSecondary ? "true" : "false");

					// Write the weight or value based on the link being a string link or not
					if (link->isStringLink())
						stream.writeTextElement("value", link->value());
					else
						stream.writeTextElement("weight", QString::number(link->weight()));
					// Be careful to use the box's uuid and not the slot's
					stream.writeTextElement("from", link->from()->box()->uuid().toString());

					// Write the connecvitity if the link is of type MATRIX_MATRIX
					if (link->to()->inputType() == MATRIX_MATRIX) {
						stream.writeStartElement("connectivity");
						stream.writeAttribute("type", connectivityToString(link->connectivity()));
						stream.writeEndElement(); // connectivity
					}

					stream.writeEndElement(); // link
				}

				stream.writeEndElement(); // links
				stream.writeEndElement(); // input
			}

			stream.writeEndElement(); // inputs
		}

		// Save output slot
		stream.writeStartElement("output");
		stream.writeAttribute("type", outputTypeToString(item->outputType()));
		// If the function outputs a matrix, write the dimensions
		if (item->outputType() == MATRIX) {
			stream.writeTextElement("rows", QString::number(item->rows()));
			stream.writeTextElement("cols", QString::number(item->cols()));
		}
		stream.writeEndElement(); // output

		stream.writeStartElement("position");
		stream.writeTextElement("x", QString::number(pos.x()));
		stream.writeTextElement("y", QString::number(pos.y()));
		stream.writeEndElement(); // position

		if (!constant)
			stream.writeTextElement("description", descriptionFile);
		stream.writeTextElement("icon", iconFilepath);

		stream.writeEndElement(); // function
	}

	stream.writeEndElement(); // functions

	stream.writeStartElement("zones");
	foreach (Zone *zone, zones) {
		stream.writeStartElement("zone");
		QPointF zonePos = zone->scenePos();
		stream.writeTextElement("title", zone->title());
		stream.writeTextElement("x", QString::number(zonePos.x()));
		stream.writeTextElement("y", QString::number(zonePos.y()));
		stream.writeTextElement("width", QString::number(zone->width()));
		stream.writeTextElement("height", QString::number(zone->height()));
		stream.writeStartElement("color");

		stream.writeAttribute("red", QString::number(zone->color().red()));
		stream.writeAttribute("green", QString::number(zone->color().green()));
		stream.writeAttribute("blue", QString::number(zone->color().blue()));
		stream.writeAttribute("alpha", QString::number(zone->color().alpha()));

		stream.writeEndElement(); //color
		stream.writeEndElement(); // zone
	}
	stream.writeEndElement(); // zones

	stream.writeEndDocument(); //Close the document

	file.close();

	// Encrypt the file if the option is set
	if (m_encrypt) {
		// TODO: at the moment, security is limited, because we copy the file.
		//       it should be done as a stream, all inside memory (look at ArraySource/Sink maybe)

		// Create encryptor
		CryptoPP::CBC_Mode<CryptoPP::AES>::Encryption enc;
		enc.SetKeyWithIV(reinterpret_cast<const byte *>(m_key.data()),
		                 m_key.size(),
		                 reinterpret_cast<const byte *>(m_iv.data()));
		// Encrypt the file
		CryptoPP::FileSource f(m_filePath.toStdString().c_str(), true,
		                       new CryptoPP::StreamTransformationFilter(enc,
		                                new CryptoPP::FileSink((m_filePath + ".crypted").toStdString().c_str())));

		// Remove the unencrypted file
		QFile::remove(m_filePath);
	}

	QString msg(tr("Script \"") + m_name + tr("\" saved!"));

	// Set the status as not modified
	setStatusModified(false);
	emit displayStatusMessage(msg, MSG_INFO);
}

void Script::autoSave()
{
	QMessageBox msgBox;
	msgBox.setText(QObject::tr("<strong>Not implemented yet</strong>"));

	msgBox.setInformativeText("Auto-saving is not yet implemented!");
	msgBox.setIcon(QMessageBox::Critical);

	msgBox.exec();
}

/**
 * @brief Script::updateTextStyle updates the current tab text's appearance based on whether the
 * script has been modified since last save and whether it is in invalid state
 */
void Script::updateTextStyle()
{
	// First, get the main window
	PapyrusWindow *mainWindow = getMainWindow();

	// Then get the tab widget and teh current index
	QTabWidget *tabWidget = mainWindow->ui()->tabWidget;
	if (tabWidget == NULL)
		informUserAndCrash(tr("Failed to fetch the main tabbed widget"));
	int index = tabWidget->currentIndex();

	// If the status is modified, add a '*' after the script's name and change color
	if (m_modified) {
		tabWidget->setTabText(index, m_name + "*");
		tabWidget->tabBar()->setTabTextColor(index, Qt::black);
	}
	// If the status is unmodified, remove the "*"and restore color
	else {
		tabWidget->setTabText(index, m_name);
		QColor color(Qt::gray);
		tabWidget->tabBar()->setTabTextColor(index, color.dark());
	}

	// And finally, change the color to red if the script is in invalid state
	if (m_isInvalid) {
		tabWidget->tabBar()->setTabTextColor(index, Qt::red);
	}
}

QString Script::name() const
{
	return m_name;
}

void Script::setName(const QString &name)
{
	m_name = name;

	// Also set the ROS node name
	if (m_rosSession != NULL)
		m_rosSession->setNodeName(QString("/kheops_%1").arg(m_name));
}

QString Script::filePath() const
{
	return m_filePath;
}

void Script::setFilePath(const QString &filePath)
{
	m_filePath = filePath;
}

DiagramScene *Script::scene() const
{
	return m_scene;
}

bool Script::modified() const
{
	return m_modified;
}

void Script::setStatusModified(bool isModified)
{
	// Prevent doing anything if status is the same
	if (m_modified == isModified)
		return;

	if (m_modifiedNotifTimer == NULL)
		informUserAndCrash(tr("The timer to warn about modified and unsaved script is null."));

	// If there was a change and it's true, check if a timer for warning exists and set one otherwise
	if (isModified) {
		if (m_modifiedNotifTimer->isActive()) {
		} else {
			m_modifiedNotifTimer->start(TIME_WARN_MODIFIED * 60 * 1000);
		}
	} else {
		m_modifiedNotifTimer->stop();
	}

	m_modified = isModified;

	updateTextStyle();
}

bool Script::isInvalid() const
{
	return m_isInvalid;
}

void Script::setIsInvalid(bool isInvalid)
{
	m_isInvalid = isInvalid;
	updateTextStyle();
}

double Script::timeValue() const
{
	return m_timeValue;
}

void Script::setTimeValue(double timeValue)
{
	m_timeValue = timeValue;
}

TimeUnit Script::timeUnit() const
{
	return m_timeUnit;
}

void Script::setTimeUnit(const TimeUnit &timeUnit)
{
	m_timeUnit = timeUnit;
}

QUuid Script::uuid() const
{
	return m_uuid;
}

void Script::setUuid(const QUuid &uuid)
{
	m_uuid = uuid;
}

void Script::warnAboutModifiedScript()
{
	QString title("\"" + m_name + "\"" + tr(" was not saved for ") + QString::number(TIME_WARN_MODIFIED) + tr(" minutes!"));
	QString msg(tr("You should save it to prevent data loss."));
	m_scene->mainWindow()->getTrayIcon()->showMessage(title, msg, QSystemTrayIcon::Warning);
}

bool Script::isActiveScript() const
{
	return m_isActiveScript;
}

void Script::setIsActiveScript(bool isActiveScript)
{
	m_isActiveScript = isActiveScript;
}

void Script::onROSSessionMessage(const QString &msg, MessageUrgency urgency)
{
	// We only re-emit the event if we are the active script
	if (m_isActiveScript)
		emit displayStatusMessage(msg, urgency);
}

void Script::onScriptResumed()
{
	// We only re-emit the event if we are the active script
	if (m_isActiveScript)
		emit scriptResumed();
}

void Script::onScriptPaused()
{
	// We only re-emit the event if we are the active script
	if (m_isActiveScript)
		emit scriptPaused();
}

void Script::onScriptStopped()
{
	// We only re-emit the event if we are the active script
	if (m_isActiveScript)
		emit scriptStopped();
}

void Script::onTimeElapsed(int h, int m, int s, int ms)
{
	// We only re-emit the event if we are the active script
	if (m_isActiveScript)
		emit timeElapsed(h, m, s, ms);
}

ROSSession *Script::rosSession() const
{
	return m_rosSession;
}

void Script::setRosSession(ROSSession *rosSession)
{
	m_rosSession = rosSession;
}

bool Script::encrypt() const
{
	return m_encrypt;
}

void Script::setEncrypt(bool encrypt)
{
	m_encrypt = encrypt;
}
