#ifndef SCRIPT_H
#define SCRIPT_H

#include "diagramscene.h"

#include <QString>
#include <QFile>

// Forward declaration because of recursive includes
class DiagramScene;

/**
 * @brief The Script class represents a neural scripts. An XML file is associated to it,
 * as well as the QGraphicsScene that contains its functions boxes
 */

class Script : public QObject
{
    Q_OBJECT
public:
    Script(DiagramScene *scene, const QString &name = "");

    void save();
    void autoSave();

    QString name() const;
    void setName(const QString &name);

    QString filePath() const;
    void setFilePath(const QString &filePath);

    DiagramScene *scene() const;

private:
    QString m_name;        // Pretty name of the script (to display in tabs for instance)
    QString m_filePath;    // Path of the (XML) file in which to save this script
    DiagramScene *m_scene; // The associated scene for this script
signals:
    void displayStatusMessage(const QString &text);
};

#endif // SCRIPT_H
