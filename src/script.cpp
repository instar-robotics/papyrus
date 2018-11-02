#include "script.h"
#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "outputslot.h"
#include "helpers.h"
#include "constants.h"
#include "constantdiagrambox.h"
#include "zone.h"
#include "hieroglyph/SimpleCmd.h"

#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QXmlStreamWriter>
#include <QDebug>
#include <QGraphicsView>
#include <QTemporaryFile>
#include <QProcess>
#include <QTimer>

#include <iostream>
#include <fstream>
#include <unistd.h>

#include <cryptopp/filters.h>
#include <cryptopp/aes.h>
#include <cryptopp/modes.h>
#include <cryptopp/osrng.h>
#include <cryptopp/hex.h>
#include <cryptopp/files.h>

#include <ros/ros.h>


Script::Script(DiagramScene *scene, const QString &name) : m_scene(scene),
                                                           m_hasTab(false),
                                                           m_rosSession(nullptr),
                                                           m_nodeName(QString("/kheops_%1").arg(name)),
                                                           m_modified(false),
                                                           m_isInvalid(false),
                                                           m_timeValue(10.0),
                                                           m_timeUnit(HZ),
                                                           m_encrypt(false),
                                                           m_isActiveScript(false),
                                                           m_isRunning(false),
                                                           m_isPaused(false)
{
	if (scene != NULL) {
		scene->setScript(this);
	}

	// We use setName() and not initialize with m_name(name) because of sanitation that must happen
	setName(name);

	m_uuid = QUuid::createUuid();

	m_modifiedNotifTimer = new QTimer(this);
	connect(m_modifiedNotifTimer, SIGNAL(timeout()), this, SLOT(warnAboutModifiedScript()));

	// Create the ROS session only if we have a name (otherwise there's no valid node name)
	if (!m_name.isEmpty())
		setupROSSession();
}

Script::~Script()
{
	if (m_rosSession != nullptr) {
		m_rosSession->setShouldQuit(true);
		m_rosSession->wait(500);

		delete m_rosSession;
	}
}

/**
 * @brief Save the script in its file, this means serializing the contents of the scene to the
 * XML file.
 * @param descriptionPath: the path to the (release or debug) description path, so that we can
 * strip it off of paths to save everything in relative.
 * @param basePath: the directory in which to open the file dialogs (used as a convenience to the
 * user to re-open the last visited directory to avoid traversing the file hierarchy over and over)
 * @param isAutoSave: if this is an autosave, allow saving even in invalid state, make it non-
 * interactive (do not open dialogs) and save in ".autosave" file
 */
void Script::save(const QString &descriptionPath, const QString &basePath, bool isAutoSave)
{
	QString scriptfilePath;

	// Allow saving in invalid state if this is an auto save
	if (!isAutoSave) {
		// Prevent saving when the script doesn't have a name
		if (m_name == NEW_SCRIPT_DEFAULT_NAME) {
			QMessageBox::warning(nullptr, tr("Can't save unnamed script."),
			                     tr("You script is still named \"") +
			                     QString(NEW_SCRIPT_DEFAULT_NAME) +
			                     tr(".\n Rename it and try saving again"));

			return;
		}
		// Prevent saving when the script is in invalid state
		if (m_isInvalid) {
			QMessageBox::warning(NULL, tr("Saving not allowed in invalid state!"),
			                     tr("You cannot save the script at this time because it is currently "
			                        "in an invalid state.\nThis can mean that:\n"
			                        "  - some function boxes are linked with SCALAR_MATRIX but are "
			                        "not the same size\n"
			                        "  - some function boxes have negative sizes\n"
			                        "  - etc."));
			return;
		}
		emit displayStatusMessage(tr("Saving \"") + m_name + "\"...", MSG_INFO);
	} else {
		emit displayStatusMessage(tr("Auto saving \"") + m_name + "\"...", MSG_INFO);
	}

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
		if (!isAutoSave) {
			emit displayStatusMessage(QObject::tr("No file for ") + m_name +
			                          tr(", please select one..."), MSG_INFO);

			QString savePath = QFileDialog::getSaveFileName(NULL,
			                             QObject::tr("Save as..."),
			                             basePath + "/" + mkFilenameFromScript(m_name.replace(' ', '_')),
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
		} else {
			// It should not happen (auto save should not be triggered when there's not filepath)
			// but just in case, display a message
			emit displayStatusMessage(tr("Auto save cancelled: no filepath."), MSG_WARNING);
			return;
		}
	}

	// At this point, we should have a filePath
	Q_ASSERT (!m_filePath.isEmpty());

	// Open a temporary file in which to save the script
	QTemporaryFile file;
	if (!file.open()) {
		//		QMessageBox::warning(NULL, QObject::tr("Could not open XML file"),
		//		                     QObject::tr("An error occured whlie trying to open the XML file to "
		//		                                 "save the script. "));
		QMessageBox::warning(nullptr, QObject::tr("Could not open temporary file"),
		                     QObject::tr("An error occured while trying to open a temporary file to "
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
		QString title = item->title();
		QPointF pos = item->scenePos();
		QUuid uuid = item->uuid();
		QString descriptionFile = item->descriptionFile();
		QString iconFilepath = item->iconFilepath();
		std::vector<InputSlot *>inputSlots = item->inputSlots();
		bool constant = (constantItem != nullptr);

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
		stream.writeTextElement("title", title);

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

					// Secondary is enforced for links that are connected to the same box
					bool isSecondary = link->to()->box() == link->from()->box();
					isSecondary |= link->secondary(); // But it can also be manually specified

					// Write attribute "constant" when the originating box is constant
					// We don't explicitly write "false" because this is rare and don't want to clutter
					if (dynamic_cast<ConstantDiagramBox *>(link->from()->box()) != nullptr)
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

	// Now that we have successfully saved the script in the temporary file, replace old script file
	// with the temporary file
	scriptfilePath = m_filePath;
	if (isAutoSave)
		scriptfilePath = m_filePath + ".autosave";

	QFile::remove(scriptfilePath);
	file.copy(scriptfilePath);

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
		CryptoPP::FileSource f(scriptfilePath.toStdString().c_str(), true,
		                       new CryptoPP::StreamTransformationFilter(enc,
		                                new CryptoPP::FileSink((scriptfilePath + ".crypted").toStdString().c_str())));

		// Remove the unencrypted file
		QFile::remove(scriptfilePath);
	}

	QString msg;
	if (!isAutoSave)
		msg = (tr("Script \"") + m_name + tr("\" saved!"));
	else
		msg = (tr("Autosaved script \"") + m_name + "\".");

	// If it was a manual save, set the script as modified and erase .autosave file
	if (!isAutoSave) {
		setStatusModified(false);
		QFile::remove(scriptfilePath + ".autosave");
	}

	emit displayStatusMessage(msg, MSG_INFO);
}

/**
 * @brief Script::updateTextStyle updates the current tab text's appearance based on whether the
 * script has been modified since last save and whether it is in invalid state
 */
void Script::updateTextStyle()
{
	// Do not attempt to modify the text style if we have not been added to a tab yet
	if (!m_hasTab)
		return;

	// First, get the main window
	PapyrusWindow *mainWindow = getMainWindow();

	// Then get the tab widget and the current index
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
	m_nodeName = QString("/kheops_%1").arg(sanitizeTopicName(m_name));

	// And then (re)create a ROSSession with the new name
	if (m_rosSession != nullptr) {
		m_rosSession->setShouldQuit(true);
		m_rosSession->wait(500);
		delete m_rosSession;
	}

	setupROSSession();
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

QString Script::nodeName() const
{
	return m_nodeName;
}

void Script::setNodeName(const QString &nodeName)
{
	m_nodeName = nodeName;
}

bool Script::isPaused() const
{
	return m_isPaused;
}

void Script::setIsPaused(bool isPaused)
{
	m_isPaused = isPaused;
}

bool Script::isRunning() const
{
	return m_isRunning;
}

void Script::setIsRunning(bool isRunning)
{
	m_isRunning = isRunning;
}

bool Script::hasTab() const
{
	return m_hasTab;
}

void Script::setHasTab(bool hasTab)
{
	m_hasTab = hasTab;
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
	// Update state
	m_isRunning = true;
	m_isPaused = false;

	// We only re-emit the event if we are the active script
	if (m_isActiveScript)
		emit scriptResumed();
}

void Script::onScriptPaused()
{
	// Update state
	m_isRunning = true;
	m_isPaused = true;

	// We only re-emit the event if we are the active script
	if (m_isActiveScript)
		emit scriptPaused();
}

void Script::onScriptStopped()
{
	// Update state
	m_isRunning = false;
	m_isPaused = false;

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

/**
 * @brief Script::runOrPause calls pause() or run() according to the internal status of the script
 */
void Script::runOrPause()
{
	if (m_isRunning && !m_isPaused)
		pause();
	else
		run();
}

/**
 * @brief Script::run launches a script or ask the script to resume execution if it is already
 * launched
 */
void Script::run()
{
	// Make sure the ROS master is up before trying to run
	if (!ros::master::check()) {
		emit displayStatusMessage(tr("Command cancelled because the ROS master is not up."), MSG_ERROR);
		return;
	}

	// Make sure we have a node name
	if (m_nodeName.isEmpty()) {
		emit displayStatusMessage(tr("No node name: cannot run"), MSG_ERROR);
		return;
	}

	// Make sure we have a ROSSession
	if (m_rosSession == nullptr) {
		emit displayStatusMessage(tr("Cannot launch script: no ROSSession"), MSG_ERROR);
		return;
	}

	// Save the node before launching it
	PapyrusWindow *mainWin = getMainWindow();
	save(mainWin->getDescriptionPath(), mainWin->lastDir());

	// Check if the save worked
	if (m_modified) {
		emit displayStatusMessage(tr("You need to provide a file to save the script in order to run it"),
		                          MSG_WARNING);
		return;
	}

	// If the node is not running, we need to launch a kheops instance
	if (!m_isRunning) {
		// Do not set a parent to QProcess otherwise, exiting Papyrus also exits the process!
		QProcess *kheopsNode = new QProcess;
		//*
		QString prog = "rosrun";
		QStringList args;
		args << "kheops";
		args << "kheops";
		//*/
		// TEMPORARY
		/*
			QString prog = "/home/nschoe/workspace/Qt/catkin_ws/devel/lib/kheops/kheops";
			QStringList args;
			//*/
		args << "-s";
		args << m_filePath;
		args << "-l";
		args << mainWin->getLibPath() + "/";
		emit displayStatusMessage(tr("Starting script \"") + m_name + "\"...");
		kheopsNode->start(prog, args);
	}
	// Otherwise, if the node is already running, we just have to ask it to resume execution
	else {
		ros::NodeHandle nh;
		QString srvName = m_nodeName + "/control";
		ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
		hieroglyph::SimpleCmd srv;
		srv.request.cmd = "resume";

		emit displayStatusMessage(tr("Resuming script \"") + m_name + "\"...");
		if (!client.call(srv)) {
			emit displayStatusMessage(tr("The RUN command failed."), MSG_ERROR);
		}
	}
}

/**
 * @brief Script::pause pauses a script's execution
 */
void Script::pause()
{
	// Make sure the script has a node name
	if (m_nodeName.isEmpty()) {
		emit displayStatusMessage(tr("No node name: cannot pause"), MSG_ERROR);
		return;
	}

	QString srvName = m_nodeName + "/control";
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = "pause";

	if (!client.call(srv)) {
		emit displayStatusMessage(tr("The PAUSE command failed."), MSG_ERROR);
	}
}

void Script::stop()
{
	// Make sure the ROS master is up
	if (!ros::master::check()) {
		emit displayStatusMessage(tr("STOP command cancelled because the ROS master is not up."), MSG_ERROR);
		return;
	}

	if (m_nodeName.isEmpty()) {
		emit displayStatusMessage(tr("No node name for this script: cannot stop"), MSG_ERROR);
		return;
	}

	QString srvName = m_nodeName + "/control";
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = "quit";

	if (!client.call(srv)) {
		emit displayStatusMessage(tr("The STOP command failed."), MSG_ERROR);
	}
}

/**
 * @brief Script::queryScriptStatus makes a ROS service call to the "control" endpoint, with the
 * "status" parameters to actually query the script for its status.
 * @return
 */
ScriptStatus Script::queryScriptStatus()
{
	// Make sure the ROS master is up
	if (!ros::master::check()) {
		emit displayStatusMessage(tr("Command cancelled because the ROS master is not up."), MSG_ERROR);
		return INVALID_SCRIPT_STATUS;
	}

	if (m_nodeName.isEmpty())
		informUserAndCrash(tr("Cannot query for script's status because no node name was specified."),
		                   tr("No node name specified"));

	QString srvName = m_nodeName + "/control";

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<hieroglyph::SimpleCmd>(srvName.toStdString());
	hieroglyph::SimpleCmd srv;
	srv.request.cmd = "status";

	if (client.call(srv)) {
		QString response = QString::fromStdString(srv.response.ret);;

		if (response == "run")
			return SCRIPT_RUNNING;
		else if (response == "pause")
			return SCRIPT_PAUSED;
		else
			emit displayStatusMessage(tr("Invalid response to command STATUS"), MSG_ERROR);
	} else {
		emit displayStatusMessage(tr("Command STATUS failed"), MSG_ERROR);
	}

	return INVALID_SCRIPT_STATUS;
}

/**
 * @brief Script::setupROSSession creates a ROSSession with the node name
 */
void Script::setupROSSession()
{
	m_rosSession = new ROSSession(m_nodeName);

	connect(m_rosSession, SIGNAL(scriptPaused()), this, SLOT(onScriptPaused()));
	connect(m_rosSession, SIGNAL(scriptResumed()), this, SLOT(onScriptResumed()));
	connect(m_rosSession, SIGNAL(scriptStopped()), this, SLOT(onScriptStopped()));
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
