#include "xmldescriptionreader.h"
#include "constants.h"
#include "types.h"

#include <iostream>

XmlDescriptionReader::XmlDescriptionReader(Category* category) : m_category(category)
{

}

/**
 * @brief Checks the name of the root element and set the IODevice for the Xml reader
 * @param device: the IO device from which to read the XML data
 * @return whether there was an error parsing the XMl data
 */
bool XmlDescriptionReader::read(QIODevice *device, QIcon &icon)
{
    std::cout << "=====" << std::endl << "Reading device" << std::endl;
    reader.setDevice(device);

    if (reader.readNextStartElement()) {
        if (reader.name() == XML_ROOT_ELEM) {
            std::cout << "Found description" << std::endl;
            readDescription(icon);
        } else {
            reader.raiseError(QObject::tr("Invalid description file"));
        }
    }

    std::cout << "Finish read" << std::endl << "=====" << std::endl << std::endl;

    return !reader.error();
}

void XmlDescriptionReader::readDescription(QIcon &icon)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == XML_ROOT_ELEM);

    std::cout << "Reading description" << std::endl;

    Function *function = new Function;
//    function->setIcon(icon);

    while (reader.readNextStartElement()) {
        std::cout << "--> " << qPrintable(reader.name().toString()) << std::endl;
        if (reader.name() == "name")
            readName(function);
        else if (reader.name() == "inputs")
            readInputs(function);
        else if (reader.name() == "output")
            readOutput(function);
        else {
            std::cout << "Skipping " << qPrintable(reader.name().toString()) << std::endl;
            reader.skipCurrentElement();
        }
    }

    function->setIcon(0, icon);
    function->setText(0, function->name());
    function->setSizeHint(0, QSize(LIBRARY_ICON_SIZE, LIBRARY_ICON_SIZE));

    m_category->addChild(function);
}

/**
 * @brief Parse the name of a function description
 * @param function: the function currently being parsed
 */
void XmlDescriptionReader::readName(Function *function)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "name");

    std::cout << "Reading name" << std::endl;

    QString name = reader.readElementText();

    if (name.isEmpty()) {
        std::cout << "Empty func name" << std::endl;
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

    std::cout << "Reading inputs" << std::endl;

    std::vector<InputSlot> inputs;
    while (reader.readNextStartElement()) {
        if (reader.name() == "input") {
            InputSlot inputSlot;

            if (reader.attributes().hasAttribute("allowMultiple") &&
                reader.attributes().value("allowMultiple").toString() == "true")
                inputSlot.allowMultiple = true;
            else
                inputSlot.allowMultiple = false;

            while (reader.readNextStartElement()) {
                if (reader.name() == "name")
                    readParameterName(&inputSlot);
                else if (reader.name() == "type")
                    readParameterType(&inputSlot);
                else {
                    std::cout << "\t[Input]Unknown parameter" << std::endl;
                    reader.skipCurrentElement();
                }
            }

            inputs.push_back(inputSlot);
        } else {
            std::cout << "\t[inputs] Skipping " << qPrintable(reader.name().toString()) << std::endl;
            reader.skipCurrentElement();
        }
    }

    function->setInputs(inputs);
}

void XmlDescriptionReader::readParameterName(ParameterSlot *paramSlot)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "name");

    std::cout << "Reading parameter name" << std::endl;

    QString paramName = reader.readElementText();

    if (!paramName.isEmpty()) {
        paramSlot->name = paramName;
        std::cout << "\tGot: " << qPrintable(paramName) << std::endl;
    } else {
        reader.raiseError("Empty parameter name are now allowed");
        reader.skipCurrentElement();
    }
}

void XmlDescriptionReader::readParameterType(ParameterSlot *paramSlot)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "type");

    std::cout << "Reading param type" << std::endl;

    QString paramName = reader.readElementText();

    if (!paramName.isEmpty()) {
        std::cout << "\tGot: " << qPrintable(paramName) << std::endl;

        if (paramName.toLower() == "scalar")
            paramSlot->type = Scalar;
        else if (paramName.toLower() == "vector")
            paramSlot->type = Vector;
        else if (paramName.toLower() == "matrix")
            paramSlot->type = Matrix;
        else {
            QString errStr = "Unknown input parameter type '";
            errStr += paramName;
            errStr += "'";

            reader.raiseError(errStr);
            reader.skipCurrentElement();
        }
    } else {
        std::cout << "Empty parameter type" << std::endl;
        reader.raiseError("Empty parameter type are now allowed");
        reader.skipCurrentElement();
    }
}

void XmlDescriptionReader::readOutput(Function *function)
{
    Q_ASSERT(reader.isStartElement() && reader.name() == "output");

    std::cout << "Reading output" << std::endl;

    OutputSlot outputSlot;
    while (reader.readNextStartElement()) {
        if (reader.name() == "name")
            readParameterName(&outputSlot);
        else if (reader.name() == "type")
            readParameterType(&outputSlot);
        else
            reader.skipCurrentElement();
    }
    function->setOutput(outputSlot);
}

