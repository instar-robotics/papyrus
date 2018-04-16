#include "xmldescriptionreader.h"
#include "constants.h"
#include "types.h"
#include "slot.h"

#include <iostream>
#include <QDebug>

XmlDescriptionReader::XmlDescriptionReader(Category* category) : m_category(category)
{

}

/**
 * @brief Checks the name of the root element and set the IODevice for the Xml reader
 * @param device: the IO device from which to read the XML data
 * @return whether there was an error parsing the XMl data
 */
bool XmlDescriptionReader::read(QIODevice *device, QIcon &icon, QString &descriptionFile)
{
    reader.setDevice(device);

    if (reader.readNextStartElement()) {
        if (reader.name() == XML_ROOT_ELEM) {
            readDescription(icon, descriptionFile);
        } else {
            reader.raiseError(QObject::tr("Invalid description file"));
        }
    }

    return !reader.error();
}

void XmlDescriptionReader::readDescription(QIcon &icon, QString &descriptionFile)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == XML_ROOT_ELEM);

    Function *function = new Function(descriptionFile);

    while (reader.readNextStartElement()) {
        if (reader.name() == "name")
            readName(function);
        else if (reader.name() == "inputs")
            readInputs(function);
        else if (reader.name() == "output")
            readOutput(function);
        else {
            reader.skipCurrentElement();
        }
    }

    function->setIcon(0, icon);
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

            if (!attributes.hasAttribute("multiple")) {
                reader.raiseError(QObject::tr("Missing boolean attribute 'multiple'."));
                continue;
            }

            if (attributes.value("multiple").toString() == "true")
                inputSlot->setMultiple(true);
            else
                inputSlot->setMultiple(false);

            while (reader.readNextStartElement()) {
                if (reader.name() == "name")
                    readParameterName(inputSlot);
                else if (reader.name() == "type")
                    readParameterType(inputSlot);
                else {
                    reader.skipCurrentElement();
                }
            }

//            inputs.push_back(inputSlot);
            function->addInputSlot(inputSlot);
        } else {
            reader.skipCurrentElement();
        }
    }

//    function->setInputs(inputs);
}

void XmlDescriptionReader::readParameterName(Slot *paramSlot)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "name");

    QString paramName = reader.readElementText();

    if (!paramName.isEmpty()) {
        paramSlot->setName(paramName);
    } else {
        reader.raiseError("Empty parameter name are now allowed");
        reader.skipCurrentElement();
    }
}

//*
void XmlDescriptionReader::readParameterType(OutputSlot *paramSlot)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "type");

    QString paramName = reader.readElementText();

    if (!paramName.isEmpty()) {
        if (paramName.toLower() == "scalar")
            paramSlot->setOutputType(SCALAR);
        else if (paramName.toLower() == "matrix")
            paramSlot->setOutputType(MATRIX);
        else {
            QString errStr = "Unknown ouput parameter type '";
            errStr += paramName;
            errStr += "'";

            reader.raiseError(errStr);
            reader.skipCurrentElement();
        }
    } else {
        reader.raiseError("Empty parameter type are now allowed");
        reader.skipCurrentElement();
    }
}

void XmlDescriptionReader::readParameterType(InputSlot *paramSlot)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "type");

    QString paramName = reader.readElementText();

    if (!paramName.isEmpty()) {
        if (paramName.toLower() == "scalar_scalar")
            paramSlot->setInputType(SCALAR_SCALAR);
        else if (paramName.toLower() == "simple_matrix")
            paramSlot->setInputType(SIMPLE_MATRIX);
        else if (paramName.toLower() == "scalar_matrix")
            paramSlot->setInputType(SCALAR_MATRIX);
        else if (paramName.toLower() == "matrix_matrix")
            paramSlot->setInputType(MATRIX_MATRIX);
        else {
            QString errStr = "Unknown input parameter type '";
            errStr += paramName;
            errStr += "'";

            reader.raiseError(errStr);
            reader.skipCurrentElement();
        }
    } else {
        reader.raiseError("Empty parameter type are now allowed");
        reader.skipCurrentElement();
    }
}
//*/

void XmlDescriptionReader::readOutput(Function *function)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "output");

    OutputSlot *outputSlot = new OutputSlot;
    while (reader.readNextStartElement()) {
        if (reader.name() == "name")
            readParameterName(outputSlot);
        else if (reader.name() == "type")
            readParameterType(outputSlot);
        else
            reader.skipCurrentElement();
    }
    function->setOutput(outputSlot);
}

