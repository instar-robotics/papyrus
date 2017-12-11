#ifndef SCRIPT_H
#define SCRIPT_H

#include <QString>
#include <QFile>

/**
 * @brief The Script class represents a neural scripts. An XML file is associated to it,
 * as well as the QGraphicsScene that contains its functions boxes
 */

class Script
{
public:
    Script(const QString &name);

    QString name() const;
    void setName(const QString &name);


    QString filePath() const;
    void setFilePath(const QString &filePath);

private:
    QString m_name;     // Pretty name of the script (to display in tabs for instance)
    QString m_filePath; // Path of the (XML) file in which to save this script
};

#endif // SCRIPT_H
