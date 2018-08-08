#include "script.h"
#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "outputslot.h"
#include "helpers.h"
#include "constants.h"
#include "constantdiagrambox.h"

#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QXmlStreamWriter>
#include <QDebug>
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
                                                           m_encrypt(false)
{
    if (scene != NULL) {
        scene->setScript(this);
    }

    m_uuid = QUuid::createUuid();

    m_modifiedNotifTimer = new QTimer(this);
    connect(m_modifiedNotifTimer, SIGNAL(timeout()), this, SLOT(warnAboutModifiedScript()));
}

/**
 * @brief Save the script in its file, this means serializing the contents of the scene to the
 * XML file.
 * @param descriptionPath: the path to the (release or debug) description path, so that we can
 * strip it off of paths to save everything in relative.
 */
void Script::save(const QString &descriptionPath)
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

    emit displayStatusMessage(tr("Saving \"") + m_name + "\"...");

    // Read keys if the script is specified to be encrypted
    if (m_encrypt) {
        std::string keyFile("/home/nschoe/workspace/qt/papyrus/key");
        std::string ivFile("/home/nschoe/workspace/qt/papyrus/iv");

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
        emit displayStatusMessage(QObject::tr("No file for ") + m_name + tr(", please select one..."));

        QString savePath = QFileDialog::getSaveFileName(NULL,
                                     QObject::tr("Save as..."),
                                     QDir::homePath(),
                                     QObject::tr("XML Files (*.xml)"));

        // Abort if it's empty
        if (savePath.isEmpty()) {
            QString text(tr("Cancelled saving "));
            text += name();
            emit displayStatusMessage(text);
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

    // Write all functions
    stream.writeStartElement("functions");

    // Traverse all items in the scene and store them
    foreach (QGraphicsItem *i, m_scene->items()) {
        // For some reasons, it fails with 'qgraphicsitem_cast' even though 'type()' is reimplemented
        DiagramBox *item = dynamic_cast<DiagramBox *>(i);
        if (item == NULL) {
            // Item is not a function, skip it
            continue;
        }

        ConstantDiagramBox *constantItem = dynamic_cast<ConstantDiagramBox *>(item);

        QString name = item->name();
        QPointF pos = item->scenePos();
        QUuid uuid = item->uuid();
        QString descriptionFile = item->descriptionFile();
        QString iconFilepath = item->iconFilepath();
        std::set<InputSlot *>inputSlots = item->inputSlots();
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
                    if (link->to()->inputType() == MATRIX_MATRIX)
                        stream.writeTextElement("connectivity", connectivityToString(link->connectivity()));

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
    emit displayStatusMessage(msg);
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

bool Script::encrypt() const
{
    return m_encrypt;
}

void Script::setEncrypt(bool encrypt)
{
    m_encrypt = encrypt;
}
