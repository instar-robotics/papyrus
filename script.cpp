#include "script.h"
#include "papyruswindow.h"
#include "ui_papyruswindow.h"
#include "outputslot.h"
#include "helpers.h"

#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QXmlStreamWriter>
#include <QDebug>
#include <iostream>

Script::Script(DiagramScene *scene, const QString &name) : m_scene(scene), m_name(name), m_modified(false)
{
    if (scene != NULL) {
        scene->setScript(this);
    }
}

/**
 * @brief Save the script in its file, this means serializing the contents of the scene to the
 * XML file.
 */
void Script::save()
{
    emit displayStatusMessage(tr("Saving ") + m_name + "...");

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
    stream.writeStartDocument();        // start the XML document
    stream.setAutoFormatting(true);     // Make it human-readable
    stream.writeStartElement("script"); // Write the root tag
    stream.writeTextElement("name", name());     // Write the name of the script
    stream.writeStartElement("functions");

    // Traverse all items in the scene and store them
    foreach (QGraphicsItem *i, m_scene->items()) {
        // For some reasons, it fails with 'qgraphicsitem_cast' even though 'type()' is reimplemented
        DiagramBox *item = dynamic_cast<DiagramBox *>(i);
        if (item == NULL) {
            // Item is not a function, skip it
            continue;
        }

        QString name = item->name();
        QPointF pos = item->scenePos();
        QUuid uuid = item->uuid();
        QString descriptionFile = item->descriptionFile();
        OutputSlot *outputSlot = item->outputSlot();
        std::set<InputSlot *>inputSlots = item->inputSlots();

        Q_ASSERT(!name.isEmpty());

        stream.writeStartElement("function");
        stream.writeAttribute("uuid", uuid.toString());
        stream.writeTextElement("name", name);

        // Save input slots
        stream.writeStartElement("inputs");
        foreach (InputSlot *inputSlot, inputSlots) {
            stream.writeStartElement("input");
            stream.writeAttribute("type", inputTypeToString(inputSlot->inputType()));
            stream.writeAttribute("anchor", "true/false");

            stream.writeTextElement("name", inputSlot->name());

            stream.writeStartElement("links");
            // Should loop for each connection
            stream.writeStartElement("link");
            stream.writeAttribute("secondary", "true/false");
            stream.writeAttribute("sparse", "true/false");
            stream.writeTextElement("weight", "1.0");
            stream.writeTextElement("from", "UUID");
            stream.writeTextElement("connectivity", "TODO");
            stream.writeTextElement("operator", "multiplicator/addition/etc");
            stream.writeEndElement(); // link

            stream.writeEndElement(); // links
            stream.writeEndElement(); // input
        }

        stream.writeEndElement(); // inputs

        // Save output slot
        stream.writeStartElement("output");
        stream.writeAttribute("type", outputTypeToString(item->outputType()).toLower());
        stream.writeTextElement("name", outputSlot->name());
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
        stream.writeTextElement("description", descriptionFile);

        // If this function has its output connect with some other functions, insert them
        // TODO : implement issue #2 and then check
        /*
        foreach (Arrow *link, item->startLines()) {
            QUuid targetId = link->to()->uuid();
            stream.writeTextElement("link", targetId.toString());
        }
        //*/

        stream.writeEndElement(); // function
    }

    stream.writeEndElement();

    stream.writeEndDocument(); //Close the document

    file.close();

    QString msg(tr("Script ") + m_name + tr(" saved!"));

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
    // NOTE: we should STILL udpate the last modified date, for the autosave feature.
    if (m_modified == isModified)
        return;

    m_modified = isModified;

    // First, get the main window
    PapyrusWindow *mainWindow = NULL;

    foreach (QWidget *w, qApp->topLevelWidgets()) {
        if (PapyrusWindow *mW = qobject_cast<PapyrusWindow *>(w)) {
            mainWindow = mW;
            break;
        }
    }

    // If the status is modified, add a '*' after the script's name and change color
    if (isModified) {
        if (mainWindow) {
            QTabWidget *tabWidget = mainWindow->getUi()->tabWidget;
            int index = tabWidget->currentIndex();
            tabWidget->setTabText(index, m_name + "*");
            tabWidget->tabBar()->setTabTextColor(index, Qt::black);
        }
    }
    // If the status is unmodified, remove the "*"and restore color
    else {
        if (mainWindow) {
            QTabWidget *tabWidget = mainWindow->getUi()->tabWidget;
            int index = tabWidget->currentIndex();
            tabWidget->setTabText(index, m_name);
            QColor color(Qt::gray);
            tabWidget->tabBar()->setTabTextColor(index, color.dark());
        }
    }
}
