#include "script.h"

#include <QMessageBox>

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
    QMessageBox msgBox;
    msgBox.setText(QObject::tr("<strong>Not implemented yet</strong>"));

    msgBox.setInformativeText("Saving is not yet implemented!");
    msgBox.setIcon(QMessageBox::Critical);

    msgBox.exec();
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
