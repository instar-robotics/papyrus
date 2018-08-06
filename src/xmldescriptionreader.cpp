#include "xmldescriptionreader.h"
#include "constants.h"
#include "slot.h"
#include "helpers.h"

#include <iostream>
#include <QDebug>
#include <QFileInfo>
#include <QDir>

XmlDescriptionReader::XmlDescriptionReader(Category* category) : m_category(category)
{

}

/**
 * @brief Checks the name of the root element and set the IODevice for the Xml reader
 * @param device: the IO device from which to read the XML data
 * @return whether there was an error parsing the XMl data
 */
bool XmlDescriptionReader::read(QIODevice *device, const QString &descriptionFile)
{
    reader.setDevice(device);

    if (reader.readNextStartElement()) {
        if (reader.name() == XML_ROOT_ELEM) {
            readDescription(descriptionFile);
        } else {
            reader.raiseError(QObject::tr("Invalid description file"));
        }
    }

    return !reader.error();
}

void XmlDescriptionReader::readDescription(const QString &descriptionFile)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == XML_ROOT_ELEM);

    if (!reader.readNextStartElement()) {
        reader.raiseError(QObject::tr("Empty description"));
        qWarning() << "Empty description";
        return;
    }

    // Read the <libname>
    if (reader.name() != "libname") {
        reader.raiseError(QObject::tr("Missing <libname> tag"));
        qWarning() << "Missing <libname> tag";
        return;
    }

    QString libName = reader.readElementText();

    // Read the <functions>
    if (!reader.readNextStartElement() || reader.name() != "functions") {
        reader.raiseError(QObject::tr("Missing <functions> tag"));
        qWarning() << "Missing <functions> tag";
        return;
    }

    readAllFunctions(libName, descriptionFile);
}

void XmlDescriptionReader::readAllFunctions(const QString &libName, const QString &descriptionFile)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "functions");

    while (reader.readNextStartElement()) {
        if (reader.name() == "function")
            readOneFunction(libName, descriptionFile);
        else
            reader.skipCurrentElement();
    }
}

void XmlDescriptionReader::readOneFunction(const QString &libName, const QString &descriptionFile)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "function");

    Function *function = new Function(descriptionFile);
    function->setLibName(libName);
    QString iconFilename;


    while (reader.readNextStartElement()) {
        if (reader.name() == "name") {
            readName(function);
        } else if (reader.name() == "inputs") {
            readInputs(function);
        } else if (reader.name() == "output") {
            readOutput(function);
        } else if (reader.name() == "icon") {
            iconFilename = readIcon();
        } else {
            qWarning() << "Skipping unsupported tag <" << reader.name() << ">";
            reader.skipCurrentElement();
        }
    }

    // After we're done parsing the function, we check if we found an icon, and if it exists,
    // otherwise we replace it with the missnig-icon function
    if (iconFilename.isEmpty()) {
        function->setIcon(0, QIcon(":/icons/icons/missing-icon.svg"));
        function->setIconFilepath(":/icons/icons/missing-icon.svg");
    }
    else {
        // Get the current directory from the description file, and search an 'icons/' directory
        QFileInfo info(descriptionFile);
        QDir dir(info.absoluteDir());
        QString iconsDir = dir.absoluteFilePath("icons");

        // Check if the designated icon exists
        QFileInfo iconInfo(iconsDir + "/" + iconFilename);
        if (iconInfo.exists()) {
            function->setIcon(0, QIcon(iconsDir + "/" + iconFilename));
            function->setIconFilepath(iconsDir + "/" + iconFilename);
        } else {
            qWarning() << "Missing icon " << iconInfo.absoluteFilePath() << ", setting missing icon"
                                                                            " instead";
            function->setIcon(0, QIcon(":/icons/icons/missing-icon.svg"));
            function->setIconFilepath(":/icons/icons/missing-icon.svg");
        }
    }

    function->setText(0, function->name());
    function->setSizeHint(0, QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));

    // Only add the function to the library (category) if there was no error while parsing it.
    if (!reader.error())
        m_category->addChild(function);
}

/**
 * @brief Parse the name of a function description
 * @param function: the function currently being parsed
 */
void XmlDescriptionReader::readName(Function *function)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "name");

    QString name = reader.readElementText();

    if (name.isEmpty()) {
        reader.raiseError("Empty function names are not allowed");
        reader.skipCurrentElement();
    }

    function->setName(name);
}

/**
 * @brief Parse the inputs of a function descriptions
 * @param function: the function currently being parsed
 */
void XmlDescriptionReader::readInputs(Function *function)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "inputs");

    while (reader.readNextStartElement()) {
        if (reader.name() == "input") {
            InputSlot *inputSlot = new InputSlot;
            QXmlStreamAttributes attributes = reader.attributes();

            // Read the 'multiple' attribute
            if (!attributes.hasAttribute("multiple")) {
                qWarning() << "Missing boolean attribute 'multiple'";
                reader.raiseError(QObject::tr("Missing boolean attribute 'multiple'."));
                continue;
            }

            if (attributes.value("multiple").toString() == "true")
                inputSlot->setMultiple(true);
            else if (attributes.value("multiple").toString() == "false")
                inputSlot->setMultiple(false);
            else {
                qWarning() << "Invalid attribute value 'multiple'";
                reader.raiseError(QObject::tr("Invalid attribute value 'multiple'"));
            }

            // Read the 'type' attribute
            if (!attributes.hasAttribute("type")) {
                qWarning() << "Missing attribute 'type'";
                reader.raiseError(QObject::tr("Missing attribute 'type'"));
                continue;
            }

            inputSlot->setInputType(stringToInputType(attributes.value("type").toString()));

            // Read the <name> tag
            while (reader.readNextStartElement()) {
                if (reader.name() == "name")
                    readParameterName(inputSlot);
                else
                    reader.skipCurrentElement();
            }

            function->addInputSlot(inputSlot);
        } else {
            reader.skipCurrentElement();
        }
    }
}

QString XmlDescriptionReader::readIcon()
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "icon");

    return reader.readElementText();
}

void XmlDescriptionReader::readParameterName(Slot *paramSlot)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "name");

    QString paramName = reader.readElementText();

    if (!paramName.isEmpty()) {
        paramSlot->setName(paramName);
    } else {
        qWarning() << "Empty parameter name are not allowed";
        reader.raiseError("Empty parameter name are not allowed");
        reader.skipCurrentElement();
    }
}

void XmlDescriptionReader::readOutput(Function *function)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "output");

    OutputSlot *outputSlot = new OutputSlot;

    QXmlStreamAttributes attributes = reader.attributes();
    if (!attributes.hasAttribute("type")) {
        qWarning() << "<output> tag is missing its 'type' attribute";
        reader.raiseError();
        return;
    }

    outputSlot->setOutputType(stringToOutputType(attributes.value("type").toString()));
    function->setOutput(outputSlot);

    // Nothing to read, but it's just to consume the <output> element
    while (reader.readNextStartElement());
}

