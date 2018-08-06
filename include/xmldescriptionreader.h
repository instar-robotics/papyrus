#ifndef XMLDESCRIPTIONREADER_H
#define XMLDESCRIPTIONREADER_H

#include "category.h"
#include "slot.h"

#include <QXmlStreamReader>

/**
 * @brief The XmlDescriptionReader class is a XML file parser whose task is to populate a @Library
 * object with the @Categorie  and descriptions of neural @Function s, read from their XML description
 * file.
 */

class XmlDescriptionReader
{
public:
    XmlDescriptionReader(Category *category );

    bool read(QIODevice *device, const QString &descriptionFile);
private:
//    Library *m_library;
    Category *m_category;
    QXmlStreamReader reader;

    void readDescription(const QString &descriptionFile);
    void readAllFunctions(const QString &libName, const QString &descriptionFile);
    void readOneFunction(const QString &libName, const QString &descriptionFile);
    void readName(Function *function);
    void readInputs(Function *function);
    QString readIcon();
    void readParameterName(Slot *paramSlot);
    void readParameterType(OutputSlot *paramSlot);
    void readParameterType(InputSlot *paramSlot);
    void readOutput(Function *function);
};

#endif // XMLDESCRIPTIONREADER_H
