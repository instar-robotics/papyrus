#include "script.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QXmlStreamWriter>

Script::Script(const QString &name, DiagramScene *scene) : m_name(name),
                                                           m_scene(scene)
{

}

/**
 * @brief Save the script in its file, this means serializing the contents of the scene to the
 * XML file.
 */
void Script::save()
{
    // First check if we have a filepath in which to save the script
    if (m_filePath.isEmpty()) {
        emit displayStatusMessage(QObject::tr("No file for this script, please select one..."));
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
            // This case means the user entered an extesnion that is not XML so fail
            QMessageBox::warning(NULL, QObject::tr("Only .xml file are supported"),
                                 QObject::tr("Scripts must be saved in .xml files, please select or "
                                             "enter a valid .xml filename."));
            return;
        }

        setFilePath(savePath);
        save();
    }
    // If we have a save path, proceed to save
    else {
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

        // Traverse all items in the view and store them
        foreach (QGraphicsItem *i, m_scene->items()) {
            DiagramBox *item = qgraphicsitem_cast<DiagramBox *>(i);
            if (!item)
                continue;

            QString name = item->name();
            QPointF pos = item->scenePos();

            stream.writeStartElement("function");
            stream.writeTextElement("name", name);
            stream.writeStartElement("position");
            stream.writeTextElement("x", QString::number(pos.x()));
            stream.writeTextElement("y", QString::number(pos.y()));
            stream.writeEndElement(); // position
            stream.writeEndElement(); // function
        }

        stream.writeEndElement();

        stream.writeEndDocument(); //Close the document

        file.close();

        QMessageBox::information(NULL, "Saved", "Saved!");
    }
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
